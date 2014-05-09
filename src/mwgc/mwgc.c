/*******************************************************************************
 derivative work from the mavlink tutorial.

 Copyright (C) 2013  Trey Marc ( a t ) gmail.com
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 -2012.04.29 : mwgc demo code
 -2013.12.18 : update to msp 2.3 + refactor code
 -2014.04.25 : send pid values to groundstation

 ****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>

// mavlink message headers
#include "common/mavlink.h"

// udp & socket
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define BUFFER_LENGTH 2048
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;
#if defined( _WINDOZ )
#include <winsock.h>
#define SocketErrno (WSAGetLastError())
#define bcopy(src,dest,len) memmove(dest,src,len)
typedef uint32_t socklen_t;
#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <err.h>
typedef int SOCKET;
#define closesocket(s) close(s)
#endif

#define FLOAT_TO_INT(VVALUIN) (VVALUIN + 0.5f)

// mwi lib
#include "../utils/utils.h"
#include "../mwi/mwi.h"

// this header
#include "mwgc.h"

enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXBARO,
    BOXVARIO,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXCALIB,
    BOXGOV,
    BOXOSD,
    BOXTELEMETRY,
    CHECKBOXITEMS
};
struct box_t {
    const uint8_t boxIndex;
    const char *boxName;
    const uint8_t permanentId;
} mwiGuiBoxes[] = {
    { BOXARM, "ARM", 0 },
    { BOXANGLE, "ANGLE", 1 },
    { BOXHORIZON, "HORIZON", 2 },
    { BOXBARO, "BARO", 3 },
    { BOXVARIO, "VARIO", 4 },
    { BOXMAG, "MAG", 5 },
    { BOXHEADFREE, "HEADFREE", 6 },
    { BOXHEADADJ, "HEADADJ", 7 },
    { BOXCAMSTAB, "CAMSTAB", 8 },
    { BOXCAMTRIG, "CAMTRIG", 9 },
    { BOXGPSHOME, "GPS HOME", 10 },
    { BOXGPSHOLD, "GPS HOLD", 11 },
    { BOXPASSTHRU, "PASSTHRU", 12 },
    { BOXBEEPERON, "BEEPER", 13 },
    { BOXLEDMAX, "LEDMAX", 14 },
    { BOXLEDLOW, "LEDLOW", 15 },
    { BOXLLIGHTS, "LLIGHTS", 16 },
    { BOXCALIB, "CALIB", 17 },
    { BOXGOV, "GOVERNOR", 18 },
    { BOXOSD, "OSD SW", 19 },
    { BOXTELEMETRY, "TELEMETRY", 20 },
    { CHECKBOXITEMS, NULL, 0xFF }
};

//--
typedef struct fs_output_t {
    // gps
    double latitude;
    double longitude;
    double altitude;
    double groundspeed;

    // Attitude
    double heading;
    double roll;
    double pitch;

    // Imu (not used)
    double accelX;
    double accelY;
    double accelZ;

    double rateRoll;
    double ratePitch;
    double rateYaw;

    // CTRL
#define FS_OUTCTRL 0xa636a53d
    uint32_t ctrl;
} fs_output_t;

typedef struct fs_input_t {
    double aileron;
    double elevator;
    double rudder;
    double throttle1;
    double throttle2;
    double throttle3;
    double throttle4;
} fs_input_t;
//--

#define PI 3.1415926535897932384626433832795
#define deg2radian(X) (PI * X) / 180

void sendMavParam(float value, const char *name, int index, int indexMax);  // send a param to gc
void handleMessage(mavlink_message_t* currentMsg);                          // handle incoming udp mavlink msg
void callBack_mwi(int state);                                               // handle incoming msp event

void swap64(void *p);
void swap32(void *p);
void sendToFS(mwi_mav_t *mwi);
void recieveFromFS(void);
void recieveFromGS(void);
void annex(void);

mavlink_state_t *mavlinkState;                      // mavlink ground station
mwi_mav_t *mwiState;                                // mwi state
mavlink_message_t msg;                              // mavlink message

HANDLE serialLink = 0;
msp_payload_t *payload;
fs_output_t fsMsg;

uint8_t buf[BUFFER_LENGTH];                         // udp buffer
SOCKET sock;                                        // udp socket
SOCKADDR_IN locGSAddr;                      // ground station
int sizeGroundStationAddr = sizeof locGSAddr;

uint8_t bufFS[BUFFER_LENGTH];                       // udp buffer
SOCKET sockFSin;                                      // udp socket
SOCKADDR_IN locFSAddr;                                 // flight simulator
int sizeFsAddr = sizeof locFSAddr;

SOCKET sockFSout;
SOCKADDR_IN locAddr;

//--
void swap32(void *p)
{
    *(int32_t *)p = htonl(*(int32_t *)p);
}

union temp64 {
    int64_t ll;
    int32_t l[2];
};

void swap64(void *p)
{
    union temp64 *f, t;

    f = (union temp64 *)p;

    t.l[0] = htonl(f->l[1]);
    t.l[1] = htonl(f->l[0]);

    f->ll = t.ll;
}
//--

int main(int argc, char* argv[])
{
#if defined( _WINDOZ )
    WSADATA WSAData;
    WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

    // msp paylaod
    payload = calloc(sizeof(*payload), sizeof(*payload));

    // mwi state
    mwiState = calloc(sizeof(*mwiState), sizeof(*mwiState));
    mwiState->callback = &callBack_mwi;

    // mavlink state
    mavlinkState = calloc(sizeof(*mavlinkState), sizeof(*mavlinkState));
    mavlinkState->mwiUavID = 1;
    mavlinkState->mwiAutoPilotType = MAV_AUTOPILOT_GENERIC;
    mavlinkState->mwiFlightMode = MAV_STATE_UNINIT; // initial mode is unknown
    mavlinkState->mwiAirFrametype = MAV_TYPE_GENERIC;
    mavlinkState->autoTelemtry = NOK;
    mavlinkState->baudrate = SERIAL_115200_BAUDRATE;
    mavlinkState->hertz = 30;
    strcpy(mavlinkState->targetIp, "127.0.0.1");
    strcpy(mavlinkState->serialDevice, "/dev/ttyO2");

    // some counters
    uint64_t lastFrameRequest = 0;
    uint64_t lastHeartBeat = 0;
    uint64_t lastReaquestLowPriority = 0;
    uint64_t currentTime = microsSinceEpoch();

    // create a configuration from command line
    if (config(mavlinkState, argc, argv) == NOK) {
        eexit(serialLink);
    }
    // translate argument to something mavlink knows about
    if (mavlinkState->mwiAutoPilotType == TYPE_PX4) {
        mavlinkState->mwiAutoPilotType = MAV_AUTOPILOT_PX4;
    }

    // create our socket
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockFSin = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockFSout = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_port = htons(14551);
    locAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    memset(&locGSAddr, 0, sizeof(locGSAddr));
    locGSAddr.sin_family = AF_INET;
    locGSAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    locGSAddr.sin_port = htons(14550); // TODO port number option

    // Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol
    if (NOK != bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind to port 14551 failed");
        eexit(serialLink);
    }

    memset(&locFSAddr, 0, sizeof(locFSAddr));
    locFSAddr.sin_family = AF_INET;
    locFSAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    locFSAddr.sin_port = htons(15500); // TODO port number option

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_port = htons(15501);
    locAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    // Bind the socket to port 15501 - necessary to receive packets from simulator
    if (NOK != bind(sockFSin, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind to port 15501 failed");
        eexit(serialLink);
    }

    // Attempt to make it non blocking
#if defined(_WINDOZ)
    u_long argi = 1;
    if(ioctlsocket(sock, FIONBIO, &argi )<0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        eexit(serialLink);
    }

    argi = 1;
    if(ioctlsocket(sockFSin, FIONBIO, &argi )<0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        eexit(serialLink);
    }
    argi = 1;
    if(ioctlsocket(sockFSout, FIONBIO, &argi )<0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        eexit(serialLink);
    }
#define SOCKLEN_T_INT(fromlen) ((int*)fromlen)
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) < 0) {
        printf("IMU setsockopt SO_REUSEADDR");
        eexit(serialLink);
    }
    opt = 1;
    if (setsockopt(sockFSin, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) < 0) {
        printf("IMU setsockopt SO_REUSEADDR");
        eexit(serialLink);
    }
    opt = 1;
    if (setsockopt(sockFSout, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) < 0) {
        printf("IMU setsockopt SO_REUSEADDR");
        eexit(serialLink);
    }
#else
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        eexit(serialLink);
    }
    if (fcntl(sockFSin, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sockFSin);
        eexit(serialLink);
    }
    if (fcntl(sockFSout, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sockFSout);
        eexit(serialLink);
    }
#define SOCKLEN_T_INT(fromlen) fromlen

    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        err(1, "IMU setsockopt SO_REUSEADDR");
    opt = 1;
    if (setsockopt(sockFSin, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        err(1, "IMU setsockopt SO_REUSEADDR");
    opt = 1;
    if (setsockopt(sockFSout, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        err(1, "IMU setsockopt SO_REUSEADDR");
    // end socket
#endif
    // init serial
    serialLink = MWIserialbuffer_init(mavlinkState->serialDevice, mavlinkState->baudrate);
    if (serialLink == NOK) {
        perror("error opening serial port");
        eexit(serialLink);
    }

    MW_TRACE("starting..\n")
    for (;;) {

        currentTime = microsSinceEpoch();

        if (!mavlinkState->autoTelemtry && ((currentTime - lastFrameRequest) > (1000 * (1000 / (uint32_t)(mavlinkState->hertz))))) {
            lastFrameRequest = currentTime;
            if (mwiState->init == OK) {
                if ((currentTime - lastHeartBeat) > 1000 * 500) {
                    // ~ 2hz
                    lastHeartBeat = currentTime;
                    MWIserialbuffer_askForFrame(serialLink, MSP_IDENT, payload);
                    MWIserialbuffer_askForFrame(serialLink, MSP_STATUS, payload);
                }
                if ((currentTime - lastReaquestLowPriority) > 1000 * 90) {
                    // ~10hz
                    lastReaquestLowPriority = currentTime;
                    MWIserialbuffer_askForFrame(serialLink, MSP_ANALOG, payload);
                    MWIserialbuffer_askForFrame(serialLink, MSP_COMP_GPS, payload);
                    MWIserialbuffer_askForFrame(serialLink, MSP_RAW_GPS, payload);
                }
                // ~30 hz
                MWIserialbuffer_askForFrame(serialLink, MSP_ATTITUDE, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RAW_IMU, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_ALTITUDE, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RC, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_MOTOR, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_SERVO, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_DEBUG, payload);
            } else {
                // we need boxnames for ARM BOX, HORIZON , ANGLE , GPS ..
                MWIserialbuffer_askForFrame(serialLink, MSP_IDENT, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_BOXNAMES, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RC_TUNING, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_PID, payload);
            }
            //TODO others if required
            //MSP_COMP_GPS
            //MSP_BAT

            // the ground station is sending rc data
            if (mavlinkState->rcdata.toSend == OK) {
                mavlinkState->rcdata.toSend = NOK;
                payload->length = 0;
                MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.y);
                MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.x);
                if (mavlinkState->fcType == FC_MWI8BIT) {
                    MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.r);
                    MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.z);
                } else {

                    MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.z);
                    MWIserialbuffer_Payloadwrite16(payload, mavlinkState->rcdata.r);
                }
                MWIserialbuffer_Payloadwrite16(payload, 1500);
                MWIserialbuffer_Payloadwrite16(payload, 1500);
                MWIserialbuffer_Payloadwrite16(payload, 1500);
                MWIserialbuffer_Payloadwrite16(payload, 1500);

                MWIserialbuffer_askForFrame(serialLink, MSP_SET_RAW_RC, payload);
            }

        }

        // fast rate
        annex();

        // we dont use thread ..
        usleep(1);
    }
}

void annex(void)
{

// update mwiStatewith new serial date from serialLink
    MWIserialbuffer_readNewFrames(serialLink, mwiState);

//  recieve udp from groundstation
    recieveFromGS();

    if (mavlinkState->hil) {
        //  recieve udp from flight simulator
        recieveFromFS();

        // send mwistate to fligh simulator
        sendToFS(mwiState);
    }
}

void recieveFromGS(void)
{
    ssize_t recsize;
    socklen_t fromlen;
    uint8_t udpInBuf[BUFFER_LENGTH];
    memset(udpInBuf, 0, BUFFER_LENGTH);

    recsize = recvfrom(sock, (void *)udpInBuf, BUFFER_LENGTH, 0, (SOCKADDR *)&locGSAddr, SOCKLEN_T_INT(&fromlen));
    if (recsize > 0) {
        MW_TRACE("\n")MW_TRACE(" <-- udp in GS <--\n")
        mavlink_message_t msgIn;
        mavlink_status_t status;
        for (int i = 0; i < recsize; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, udpInBuf[i], &msgIn, &status)) {
                // Packet received
                if (mavlinkState->verbose) {
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msgIn.sysid, msgIn.compid, msgIn.len, msgIn.msgid);
                }
                handleMessage(&msgIn);
            }
        }
    }
}

void recieveFromFS()
{

    int recsize = recvfrom(sockFSin, (char*)&fsMsg, sizeof(fsMsg), 0, NULL, NULL);
    if (recsize > 0) {
        MW_TRACE("\n")MW_TRACE(" <-- udp in FS <--\n")
        swap32(&fsMsg.ctrl);
        if (fsMsg.ctrl != FS_OUTCTRL) {
            if (mavlinkState->verbose) {
                printf("%d\n", fsMsg.ctrl);
            }
            MW_TRACE(" <-- udp in FS Checksum Failed <--\n")
        } else {

            /* endian-swap message fields */
            swap64(&fsMsg.latitude);
            swap64(&fsMsg.longitude);
            swap64(&fsMsg.altitude);
            swap64(&fsMsg.groundspeed);

            swap64(&fsMsg.heading);
            swap64(&fsMsg.roll);
            swap64(&fsMsg.pitch);

            swap64(&fsMsg.accelX);
            swap64(&fsMsg.accelY);
            swap64(&fsMsg.accelZ);

            swap64(&fsMsg.rateRoll);
            swap64(&fsMsg.ratePitch);
            swap64(&fsMsg.rateYaw);

            if (fsMsg.altitude < 0) {
                fsMsg.altitude = 0;
            }

            payload->length = 0;
            MWIserialbuffer_Payloadwrite16(payload, FLOAT_TO_INT(fsMsg.roll * 10.0f));
            MWIserialbuffer_Payloadwrite16(payload, FLOAT_TO_INT(-fsMsg.pitch * 10.0f));
            MWIserialbuffer_Payloadwrite16(payload, FLOAT_TO_INT(fsMsg.heading));
            MWIserialbuffer_Payloadwrite32(payload, FLOAT_TO_INT(fsMsg.altitude * 0.3048) * 100);
            MWIserialbuffer_askForFrame(serialLink, MSP_SET_ATTITUDE, payload);

            payload->length = 0;
            MWIserialbuffer_Payloadwrite8(payload, 1);          // GPS_FIX
            MWIserialbuffer_Payloadwrite8(payload, 9);          // GPS_numSat
            MWIserialbuffer_Payloadwrite32(payload, (int32_t)(fsMsg.latitude * 10000000));   // GPS_coord[LAT] / 90
            MWIserialbuffer_Payloadwrite32(payload, (int32_t)(fsMsg.longitude * 10000000));   // GPS_coord[LON]/ 180
            MWIserialbuffer_Payloadwrite16(payload, FLOAT_TO_INT(fsMsg.altitude * 0.3048) * 10);      // GPS_altitude 0.1m
            MWIserialbuffer_Payloadwrite16(payload, (int32_t)(fsMsg.groundspeed));      // GPS_speed

            MWIserialbuffer_askForFrame(serialLink, MSP_SET_RAW_GPS, payload);

        }
    } else {
        // MW_TRACE("\n")MW_TRACE(" <-- nothing to read from udp sockFSin :(  <--\n")
    }
}
void sendToFS(mwi_mav_t *mwi)
{
    fs_input_t cdata;
    if (mavlinkState->fcType == FC_MWI8BIT) {
        // PLANE default
       cdata.aileron = (double)(-(mwi->servo[4] - 1500) / 500.0f);
        cdata.elevator = (double)((mwi->servo[6] - 1500) / 500.0f);
        cdata.rudder = (double)((mwi->servo[5] - 1500) / 500.0f);
        cdata.throttle1 = (double)((mwi->mot[0] - 1000) / 1000.0f);
        cdata.throttle2 = (double)((mwi->mot[0] - 1000) / 1000.0f);
        cdata.throttle3 = (double)((mwi->mot[1] - 1000) / 1000.0f);
        cdata.throttle4 = (double)((mwi->mot[2] - 1000) / 1000.0f);
    } else {
        cdata.aileron = (double)((mwi->servo[4] - 1500) / 500.0f);
        cdata.elevator = (double)((mwi->servo[6] - 1500) / 500.0f);
        cdata.rudder = (double)(-(mwi->servo[5] - 1500) / 500.0f);
        cdata.throttle1 = (double)((mwi->mot[0] - 1000) / 1000.0f);
        cdata.throttle2 = (double)((mwi->mot[0] - 1000) / 1000.0f);
        cdata.throttle3 = (double)((mwi->mot[1] - 1000) / 1000.0f);
        cdata.throttle4 = (double)((mwi->mot[2] - 1000) / 1000.0f);
    }

    swap64(&cdata.aileron);
    swap64(&cdata.elevator);
    swap64(&cdata.rudder);
    swap64(&cdata.throttle1);
    swap64(&cdata.throttle2);
    swap64(&cdata.throttle3);
    swap64(&cdata.throttle4);

    int n;
    n = sendto(sockFSout, (const char*)&cdata, sizeof(cdata), 0, (struct sockaddr *)&locFSAddr, sizeFsAddr);
    if (n < 0)
        printf("ERROR in sockFSout");
}

#define SERVO_CHAN  1
#define MOTOR_CHAN  2
void callBack_mwi(int state)
{
    uint64_t currentTime = microsSinceEpoch();
    int i;
    int len = 0;
    int gpsHold = NOK;
    int gpsHome = NOK;
    int armed = NOK;
    int stabilize = NOK;
//   int mavMode = MAV_MODE_MANUAL_DISARMED;

    int MAVLINK_SENSOR_PRESENT_DEFAULT = (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION
            | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
    uint32_t sensors = MAVLINK_SENSOR_PRESENT_DEFAULT;
    switch (state) {
        case MSP_IDENT:

            switch (mwiState->multiType) {
                case 1:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_TRICOPTER;
                    break;
                case 2:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_QUADROTOR;
                    break;
                case 3:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_QUADROTOR;
                    break;
                case 7:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_HEXAROTOR;
                    break;
                case 10:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_HEXAROTOR;
                    break;
                case 11:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_OCTOROTOR;
                    break;
                case 12:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_OCTOROTOR;
                    break;
                case 13:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_OCTOROTOR;
                    break;
                case 14:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_FIXED_WING;
                    break;
                case 18:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_HEXAROTOR;
                    break;
                case 20:
                    mavlinkState->mwiAirFrametype = MAV_TYPE_HELICOPTER;
                    break;
            }

            for (i = 0; i < mwiState->boxcount; i++) {
                (mwiState->box[i])->state = ((mwiState->mode & (1 << i)) > 0);
                if (0 == strcmp(mwiState->box[i]->name, mwiGuiBoxes[BOXARM].boxName)) {
                    armed = mwiState->box[i]->state;
                }
                if (0 == strcmp(mwiState->box[i]->name, mwiGuiBoxes[BOXHORIZON].boxName)) {
                    stabilize = mwiState->box[i]->state;
                }
                if (0 == strcmp(mwiState->box[i]->name, mwiGuiBoxes[BOXGPSHOLD].boxName)) {
                    gpsHold = mwiState->box[i]->state;
                }
                if (0 == strcmp(mwiState->box[i]->name, mwiGuiBoxes[BOXGPSHOME].boxName)) {
                    gpsHome = mwiState->box[i]->state;
                }
            }

            if ((gpsHome || gpsHold) && armed)
                mavlinkState->mwiFlightMode = MAV_MODE_AUTO_ARMED;
            else if ((gpsHome || gpsHold) && !armed)
                mavlinkState->mwiFlightMode = MAV_MODE_AUTO_DISARMED;
            else if (armed && !stabilize)
                mavlinkState->mwiFlightMode = MAV_MODE_MANUAL_ARMED;

            else if (!armed && !stabilize)
                mavlinkState->mwiFlightMode = MAV_MODE_MANUAL_DISARMED;

            else if (armed && stabilize)
                mavlinkState->mwiFlightMode = MAV_MODE_STABILIZE_ARMED;

            else if (!armed && stabilize)
                mavlinkState->mwiFlightMode = MAV_MODE_STABILIZE_DISARMED;

            int reportedState;
//            if (mavlinkState->calibrating == TRUE) {
//                reportedState = MAV_STATE_CALIBRATING;
//                mavMode = MAV_MODE_PREFLIGHT;
//            } else {
            reportedState = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
//            }
            if (mavlinkState->sendRcData) {
                mavlinkState->mwiFlightMode = MAV_MODE_MANUAL_ARMED;
            }
            mavlink_msg_heartbeat_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, mavlinkState->mwiAirFrametype, mavlinkState->mwiAutoPilotType, mavlinkState->mwiFlightMode, 0, reportedState);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);

            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            mwiState->init = OK;

            break;

        case MSP_STATUS:
            // Send Status

            sensors |= MAV_SYS_STATUS_SENSOR_3D_MAG;
            sensors |= MAV_SYS_STATUS_SENSOR_GPS;
            sensors |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

            // all present sensors enabled by default except altitude and position control which we will set individually
            sensors = sensors & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);

            mavlink_msg_sys_status_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, sensors, sensors, 0, (mwiState->cycleTime / 10), mwiState->vBat * 100, mwiState->pAmp, mwiState->pMeterSum, mwiState->rssi, mwiState->i2cError, mwiState->debug[0], mwiState->debug[1],
                    mwiState->debug[2], mwiState->debug[3]);

            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_RAW_IMU:
            // Send raw imu
            mavlink_msg_raw_imu_pack(mavlinkState->mwiUavID, MAV_COMP_ID_IMU, &msg, currentTime, mwiState->ax, mwiState->ay, mwiState->az, mwiState->gx, mwiState->gy, mwiState->gz, mwiState->magx, mwiState->magy, mwiState->magz);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_SERVO:
            // Send servo - SERVO_CHAN
            mavlink_msg_servo_output_raw_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime, SERVO_CHAN, mwiState->servo[0], mwiState->servo[1], mwiState->servo[2], mwiState->servo[3], mwiState->servo[4], mwiState->servo[5], mwiState->servo[6], mwiState->servo[7]);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_MOTOR:
            // Send servo - MOTOR_CHAN
            mavlink_msg_servo_output_raw_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime, MOTOR_CHAN, mwiState->mot[0], mwiState->mot[1], mwiState->mot[2], mwiState->mot[3], mwiState->mot[4], mwiState->mot[5], mwiState->mot[6], mwiState->mot[7]);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_RC:
            // Send rcDate
            if (mavlinkState->fcType == FC_MWI8BIT) {
                mwiState->rcThrottle += mwiState->rcYaw;
                mwiState->rcYaw = mwiState->rcThrottle -mwiState->rcYaw;
                mwiState->rcThrottle -=   mwiState->rcYaw ;
            }


            mavlink_msg_rc_channels_raw_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime / 1000, 0, mwiState->rcRoll, mwiState->rcPitch, mwiState->rcThrottle, mwiState->rcYaw, mwiState->rcAUX1, mwiState->rcAUX2, mwiState->rcAUX3, mwiState->rcAUX4,
                                    mwiState->rssi);

            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

//            mavlink_msg_rc_channels_scaled_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime / 1000, 0, mwiState->rcRoll - 1500, mwiState->rcPitch - 1500, mwiState->rcThrottle - 1500, mwiState->rcYaw - 1500, mwiState->rcAUX1 - 1500,
//                    mwiState->rcAUX2 - 1500, mwiState->rcAUX3 - 1500, mwiState->rcAUX4 - 1500, mwiState->rssi);
//            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
//            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            mavlink_msg_rc_channels_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime / 1000, 8, mwiState->rcRoll, mwiState->rcPitch, mwiState->rcThrottle, mwiState->rcYaw, mwiState->rcAUX1, mwiState->rcAUX2, mwiState->rcAUX3, mwiState->rcAUX4, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, mwiState->rssi);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            // Send update hud
            mavlink_msg_vfr_hud_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, 0, 0, mwiState->head, (mwiState->rcThrottle - 1000) / 10, FLOAT_TO_INT(mwiState->baro / 100.0f), FLOAT_TO_INT(mwiState->vario / 100.0f));
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_RAW_GPS:
            // Send gps
            mavlink_msg_gps_raw_int_pack(mavlinkState->mwiUavID, MAV_COMP_ID_GPS, &msg, currentTime, mwiState->GPS_fix + 1, mwiState->GPS_latitude, mwiState->GPS_longitude, FLOAT_TO_INT(mwiState->GPS_altitude * 100.0f), 0, 0, mwiState->GPS_speed, 0, mwiState->GPS_numSat);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            mavlink_msg_global_position_int_pack(mavlinkState->mwiUavID, MAV_COMP_ID_GPS, &msg, currentTime / 1000, mwiState->GPS_latitude, mwiState->GPS_longitude, FLOAT_TO_INT(mwiState->GPS_altitude * 100.0f), FLOAT_TO_INT(mwiState->baro * 1000.0f), 0, 0, 0, 0);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            break;

        case MSP_COMP_GPS:
            break;

        case MSP_ATTITUDE:
            // Send attitude
            mavlink_msg_attitude_pack(mavlinkState->mwiUavID, MAV_COMP_ID_IMU, &msg, currentTime / 1000, deg2radian(FLOAT_TO_INT(mwiState->angx)), -deg2radian(FLOAT_TO_INT(mwiState->angy)), deg2radian(mwiState->head), deg2radian(mwiState->gx), deg2radian(mwiState->gy),
                    deg2radian(mwiState->gz));
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);

            // Send Local Position - unused without other sensors or gps cord
            //mavlink_msg_local_position_ned_pack(mavlinkState->mwiUavID, 200, &msg, currentTime, 0, 0, 0, 0, 0, 0);
            //len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            //sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);
            break;

        case MSP_ALTITUDE:
            break;

        case MSP_ANALOG: // TODO SEND
            break;

        case MSP_RC_TUNING:
            break;

        case MSP_ACC_CALIBRATION:
            break;

        case MSP_MAG_CALIBRATION:
            break;

        case MSP_PID:
            break;

        case MSP_BOX:
            break;

        case MSP_MISC: // TODO SEND
            break;

        case MSP_MOTOR_PINS: // TODO SEND
            break;

        case MSP_DEBUG:
            // Send extra debug values (uint32_t time_boot_ms)
            mavlink_msg_debug_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, (uint32_t)(currentTime / 1000), 1, mwiState->serialErrorsCount);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);
            break;

        case MSP_BOXNAMES:
            break;

        case MSP_PIDNAMES:
            break;

        case NOK:
            break;

    }

}

void sendMavParam(float value, const char *name, int index, int indexMax)
{
    mavlink_msg_param_value_pack(mavlinkState->mwiUavID, MAV_COMP_ID_ALL, &msg, name, value, MAV_PARAM_TYPE_REAL32, indexMax, index);
    sendto(sock, (const char *)buf, (char)mavlink_msg_to_send_buffer(buf, &msg), 0, (struct sockaddr*)&locGSAddr, sizeGroundStationAddr);
}

// gc -> fc
void handleMessage(mavlink_message_t* currentMsg)
{
    switch (currentMsg->msgid) {

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(currentMsg, &packet);

            // todo stop msp request
//            int freq = 0; // packet frequency
//            if (packet.start_stop == 0)
//                freq = 0; // stop sending
//            else if (packet.start_stop == 1)
//                freq = packet.req_message_rate; // start sending
//            else
//                break;

            // todo adjust msp request
            switch (packet.req_stream_id) {
                case MAV_DATA_STREAM_ALL:
                    // global_data.streamRateExtra3 = freq;
                    break;
                case MAV_DATA_STREAM_RAW_SENSORS:
                    // global_data.streamRateRawSensors = freq;
                    break;
                case MAV_DATA_STREAM_EXTENDED_STATUS:
                    // global_data.streamRateExtendedStatus = freq;
                    break;
                case MAV_DATA_STREAM_RC_CHANNELS:
                    // global_data.streamRateRCChannels = freq;
                    break;
                case MAV_DATA_STREAM_RAW_CONTROLLER:
                    // global_data.streamRateRawController = freq;
                    break;
                    //case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
                    //   // global_data.streamRateRawSensorFusion = freq;
                    //    break;
                case MAV_DATA_STREAM_POSITION:
                    // global_data.streamRatePosition = freq;
                    break;
                case MAV_DATA_STREAM_EXTRA1:
                    // global_data.streamRateExtra1 = freq;
                    break;
                case MAV_DATA_STREAM_EXTRA2:
                    // global_data.streamRateExtra2 = freq;
                    break;
                case MAV_DATA_STREAM_EXTRA3:
                    // global_data.streamRateExtra3 = freq;
                    break;
                default:
                    break;
            }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(currentMsg, &packet);

            // Start sending parameters
            int MWI_VALUESCOUNT = (3 * MWI_PIDITEMS) + (4 * MWI_CHAN_COUNT) + 1 + 11;  //pid + rctconf + rc_type + rcmap
            int p = 1;
            int i;
            char name[16] = "PID_ _";

            for (i = 0; i < MWI_PIDITEMS; i++) {
                name[4] = i + '0';
                name[6] = 'P';
                sendMavParam((float)mwiState->byteP[i], name, MWI_VALUESCOUNT, p++);

                name[4] = i + '0';
                name[6] = 'I';
                sendMavParam((float)mwiState->byteI[i], name, MWI_VALUESCOUNT, p++);

                name[4] = i + '0';
                name[6] = 'D';
                sendMavParam((float)mwiState->byteD[i], name, MWI_VALUESCOUNT, p++);
            }

            char rcMin[8] = "RC _MIN";
            char rcMax[8] = "RC _MAX";
            char rcTrim[9] = "RC _TRIM";
            char rcReverse[8] = "RC _REV";

            for (i = 0; i < MWI_CHAN_COUNT; i++) {
                rcMin[2] = (i + 1) + '0';
                sendMavParam((float)1000, rcMin, MWI_VALUESCOUNT, p++);

                rcMax[2] = (i + 1) + '0';
                sendMavParam((float)2000, rcMax, MWI_VALUESCOUNT, p++);

                rcTrim[2] = (i + 1) + '0';
                sendMavParam((float)1500, rcTrim, MWI_VALUESCOUNT, p++);

                rcReverse[2] = (i + 1) + '0';
                sendMavParam((float)((i == 1) ? -1 : 1), rcReverse, MWI_VALUESCOUNT, p++);
            }

            sendMavParam((float)1, "RC_TYPE", MWI_VALUESCOUNT, p++);

            sendMavParam((float)1, "RC_MAP_ROLL", MWI_VALUESCOUNT, p++);
            sendMavParam((float)2, "RC_MAP_PITCH", MWI_VALUESCOUNT, p++);
            sendMavParam((float)3, "RC_MAP_THROTTLE", MWI_VALUESCOUNT, p++);
            sendMavParam((float)4, "RC_MAP_YAW", MWI_VALUESCOUNT, p++);

            sendMavParam((float)0, "RC_MAP_AUX1", MWI_VALUESCOUNT, p++);
            sendMavParam((float)0, "RC_MAP_AUX2", MWI_VALUESCOUNT, p++);

            sendMavParam((float)0, "RC_MAP_ASSIST_SW", MWI_VALUESCOUNT, p++);
            sendMavParam((float)0, "RC_MAP_MISSIO_SW", MWI_VALUESCOUNT, p++);
            sendMavParam((float)0, "RC_MAP_MODE_SW", MWI_VALUESCOUNT, p++);
            sendMavParam((float)0, "RC_MAP_RETURN_SW", MWI_VALUESCOUNT, p++);
            sendMavParam((float)0, "RC_MAP_FLAPS", MWI_VALUESCOUNT, p++);

        }
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t packet;
            mavlink_msg_param_request_read_decode(currentMsg, &packet);
            if (mavlinkState->verbose) {
                printf("request param = %d , id = %s\n", (packet.param_index), (packet.param_id));
            }

        }
            break;

        case MAVLINK_MSG_ID_MANUAL_CONTROL:

            if (mavlinkState->sendRcData) {
                mavlink_manual_control_t packet;
                mavlink_msg_manual_control_decode(currentMsg, &packet);

                mavlinkState->rcdata.x = 1500 - packet.x / 2;
                mavlinkState->rcdata.y = 1500 + packet.y / 2;
                mavlinkState->rcdata.r = 1500 + packet.r / 2;
                if (mavlinkState->throttleHalfRange) {
                    mavlinkState->rcdata.z = 1000 + packet.z;
                } else {
                    mavlinkState->rcdata.z = 1500 + packet.z / 2;
                }

                mavlinkState->rcdata.buttons = packet.buttons;
                mavlinkState->rcdata.toSend = OK;

            }
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(currentMsg, &packet);
            if (mavlinkState->verbose) {
                printf("cmd %i\t param %i %i %i %i %i %i %i \t target %i %i \tack %u\n", (int)packet.command, (int)packet.param1, (int)packet.param2, (int)packet.param3, (int)packet.param4, (int)packet.param5, (int)packet.param6, (int)packet.param7, (int)packet.target_system,
                        (int)packet.target_component, packet.confirmation);
            }
        }

            break;

        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(currentMsg, &packet);

            // TODO set parameter
            const char * key = (const char*)packet.param_id;

            //            // find the requested parameter
            //            vp = AP_Var::find(key);
            //            if ((NULL != vp) &&                             // exists
            //                    !isnan(packet.param_value) &&               // not nan
            //                    !isinf(packet.param_value)) {               // not inf
            //
            //                // fetch the variable type ID
            //                var_type = vp->meta_type_id();
            //
            //                // handle variables with standard type IDs
            //                if (var_type == AP_Var::k_typeid_float) {
            //                    ((AP_Float *)vp)->set(packet.param_value);
            //
            //                } else if (var_type == AP_Var::k_typeid_float16) {
            //                    ((AP_Float16 *)vp)->set(packet.param_value);
            //
            //                } else if (var_type == AP_Var::k_typeid_int32) {
            //                    ((AP_Int32 *)vp)->set(packet.param_value);
            //
            //                } else if (var_type == AP_Var::k_typeid_int16) {
            //                    ((AP_Int16 *)vp)->set(packet.param_value);
            //
            //                } else if (var_type == AP_Var::k_typeid_int8) {
            //                    ((AP_Int8 *)vp)->set(packet.param_value);
            //                }
            //            }
            if (mavlinkState->verbose) {
                printf("key = %s, value = %g\n", key, packet.param_value);
            }
            //            // Report back new value
            //            char param_name[ONBOARD_PARAM_NAME_LENGTH];             // XXX HACK - need something to return a char *
            //            vp->copy_name(param_name, sizeof(param_name));
            //            mavlink_msg_param_value_send(chan,
            //            (int8_t*)param_name,
            //            packet.param_value,
            //            _count_parameters(),
            //            -1);                       // XXX we don't actually know what its index is...
        }
            break;
    }
}

void eexit(HANDLE serial)
{
#if defined( _WINDOZ)
    CloseHandle(serial);
    WSACleanup();
#else
    if (serial > 0) {
        close(serial);
    }
#endif

    if (sock > 0) {
        close(sock);
    }
    exit(EXIT_SUCCESS);
}
