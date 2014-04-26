/*******************************************************************************
 derivative work from :

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
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

// windows headers
#if defined( _WINDOZ )

#include <winsock.h>
#define SocketErrno (WSAGetLastError())
#define bcopy(src,dest,len) memmove(dest,src,len)
typedef uint32_t socklen_t;

#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <netdb.h> // gethostbyname
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

#endif

#include "../mwi/mwi.h"
#include "../include/utils.h"

#include "man.h"
/*
 * math
 */
#define PI 3.1415926535897932384626433832795
#define deg2radian(X) (PI * X) / 180

// mavlink message headers
#define  MAVLINK_EXTERNAL_RX_BUFFER 0
#include "message/common/mavlink.h"

// handle incoming udp mavlink msg
void handleMessage(mavlink_message_t* currentMsg);

void callBack_mwi(int state);

//default value
#define DEFAULT_UAVID 1
#define DEFAULT_IP_GS "127.0.0.1"
#define DEFAULT_SERIAL_DEV "/dev/ttyO2"

mwi_uav_state_t *mwiState;
int identSended = NOK;

// global : serialPort, socket , uavId, groundstation
HANDLE serialLink = 0;
int autoTelemtry = 0;
int baudrate = SERIAL_115200_BAUDRATE;
uint32_t hertz = 30;
SOCKET sock;
short mwiUavID;
int len = 0;
uint8_t buf[BUFFER_LENGTH];
mavlink_message_t msg;

SOCKADDR_IN groundStationAddr;
int sizeGroundStationAddr = sizeof groundStationAddr;

int main(int argc, char* argv[])
{
    memset(&groundStationAddr, 0, sizeGroundStationAddr);
#if defined( _WINDOZ )
    WSADATA WSAData;
    WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

    char targetIp[150];
    char serialDevice[150];

    mwiUavID = DEFAULT_UAVID;
    strcpy(serialDevice, DEFAULT_SERIAL_DEV);
    strcpy(targetIp, DEFAULT_IP_GS);

    uint8_t udpInBuf[BUFFER_LENGTH];
    struct sockaddr_in locAddr;

    ssize_t recsize;
    socklen_t fromlen;

    // Check if --help flag was used
    if ((argc == 2) && (strcmp(argv[1], "--help") == 0)) {
        rtfmHelp();
        eexit(EXIT_SUCCESS);
    }

    // Check if --version flag was used
    if ((argc == 2) && (strcmp(argv[1], "--version") == 0)) {
        rtfmVersion(MWGC_VERSION);
        eexit(EXIT_SUCCESS);
    }

    // parse other flag
    for (int i = 1; i < argc; i++) {
        if (i + 1 != argc) {
            if (strcmp(argv[i], "-ip") == 0) {
                strcpy(targetIp, argv[i + 1]);
                i++;
            } else if (strcmp(argv[i], "-s") == 0) {
                strcpy(serialDevice, argv[i + 1]);
                i++;
            } else if (strcmp(argv[i], "-id") == 0) {
                mwiUavID = atoi(argv[i + 1]);
                i++;
            } else if (strcmp(argv[i], "-autotelemetry") == 0) {
                autoTelemtry = atoi(argv[i + 1]);
                i++;
            } else if (strcmp(argv[i], "-baudrate") == 0) {
                baudrate = atoi(argv[i + 1]);
                i++;
            } else if (strcmp(argv[i], "-hertz") == 0) {
                hertz = atoi(argv[i + 1]);
                i++;
            }
        }
    }

    if (hertz > 60)
        hertz = 60;

    printf("ground station ip: %s\n", targetIp);
    printf("serial link: %s\n", serialDevice);
    printf("uav id: %i\n", mwiUavID);

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);

    // Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol
    if (NOK != bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind to port 14551 failed");
        eexit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
#if defined(_WINDOZ)
    u_long arg = 1;
    if(ioctlsocket(sock, FIONBIO, &arg )<0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        eexit(EXIT_FAILURE);
    }
#define SOCKLEN_T_INT(fromlen) ((int*)fromlen)
#else
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        eexit(EXIT_FAILURE);
    }
#define SOCKLEN_T_INT(fromlen) fromlen
#endif

    groundStationAddr.sin_family = AF_INET;
    groundStationAddr.sin_addr.s_addr = inet_addr(targetIp);
    groundStationAddr.sin_port = htons(14550);

    serialLink = MWIserialbuffer_init(serialDevice, baudrate);

    if (serialLink == NOK) {
        perror("error opening serial port");
        eexit(EXIT_FAILURE);
    }

    // init mwi state
    mwiState = calloc(sizeof(*mwiState), sizeof(*mwiState));
    mwiState->mode = MAV_MODE_MANUAL_DISARMED; // initial mode is unknow
    mwiState->callback = &callBack_mwi;

    MW_TRACE("starting..\n")

    uint64_t lastFrameRequest = 0;
    uint64_t lastHeartBeat = 0;
    uint64_t lastReaquestLowPriority = 0;
    uint64_t currentTime = microsSinceEpoch();

    msp_payload_t *payload;
    payload = calloc(sizeof(*payload), sizeof(*payload));
    for (;;) {

        currentTime = microsSinceEpoch();

        if (!autoTelemtry && ((currentTime - lastFrameRequest) > (1000 * (1000 / hertz)))) {
            lastFrameRequest = currentTime;
            if (identSended == OK) {
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
                // we need boxnames
                MWIserialbuffer_askForFrame(serialLink, MSP_IDENT, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_BOXNAMES, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RC_TUNING, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_PID, payload);

            }
            //TODO
            //MSP_COMP_GPS
            //MSP_BAT

        }

        MWIserialbuffer_readNewFrames(serialLink, mwiState);

        //  udp in
        memset(udpInBuf, 0, BUFFER_LENGTH);

        recsize = recvfrom(sock, (void *)udpInBuf, BUFFER_LENGTH, 0, (SOCKADDR *)&groundStationAddr, SOCKLEN_T_INT(&fromlen));

        if (recsize > 0) {
            MW_TRACE("\n")MW_TRACE(" <-- udp in <--\n")
            // Something received - print out all bytes and parse packet
            mavlink_message_t msgIn;
            mavlink_status_t status;
            //unsigned int temp = 0;
            //printf("Bytes Received: %d\nDatagram: ", (int)recsize);
            for (int i = 0; i < recsize; ++i) {
                //temp = udpInBuf[i];
                //printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(MAVLINK_COMM_0, udpInBuf[i], &msgIn, &status)) {
                    // Packet received
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msgIn.sysid, msgIn.compid, msgIn.len, msgIn.msgid);
                    handleMessage(&msgIn);
                }
            }
        }

        // no need to rush
        usleep(1);
    }
}

#define SERVO_CHAN  1
#define MOTOR_CHAN  2
void callBack_mwi(int state)
{

    uint64_t currentTime = microsSinceEpoch();
    int i;

    int gps = 0;
    int armed = 0;
    int stabilize = 0;
    int mavState = MAV_MODE_MANUAL_DISARMED;
    int uavtype = MAV_TYPE_GENERIC;
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)
    uint32_t sensors = MAVLINK_SENSOR_PRESENT_DEFAULT;
    switch (state) {
        case MSP_IDENT:

            switch (mwiState->multiType) {
                case 1:
                    uavtype = MAV_TYPE_TRICOPTER;
                    break;
                case 2:
                    uavtype = MAV_TYPE_QUADROTOR;
                    break;
                case 3:
                    uavtype = MAV_TYPE_QUADROTOR;
                    break;
                case 7:
                    uavtype = MAV_TYPE_HEXAROTOR;
                    break;
                case 10:
                    uavtype = MAV_TYPE_HEXAROTOR;
                    break;
                case 11:
                    uavtype = MAV_TYPE_OCTOROTOR;
                    break;
                case 12:
                    uavtype = MAV_TYPE_OCTOROTOR;
                    break;
                case 13:
                    uavtype = MAV_TYPE_OCTOROTOR;
                    break;
                case 14:
                    uavtype = MAV_TYPE_FIXED_WING;
                    break;
                case 18:
                    uavtype = MAV_TYPE_HEXAROTOR;
                    break;
                case 20:
                    uavtype = MAV_TYPE_HELICOPTER;
                    break;
            }

            for (i = 0; i < mwiState->boxcount; i++) {
                (mwiState->box[i])->state = ((mwiState->mode & (1 << i)) > 0);
                if (0 == strcmp(mwiState->box[i]->name, "ARM")) {
                    armed = mwiState->box[i]->state;
                }
                if (0 == strcmp(mwiState->box[i]->name, "HORIZON")) {
                    stabilize = mwiState->box[i]->state;
                }
            }

            if (gps && armed && stabilize)

                mavState = MAV_MODE_GUIDED_ARMED;

            else if (gps && !armed && stabilize)
                mavState = MAV_MODE_GUIDED_DISARMED;

            else if (!gps && armed && !stabilize)
                mavState = MAV_MODE_MANUAL_ARMED;

            else if (!gps && !armed && !stabilize)
                mavState = MAV_MODE_MANUAL_DISARMED;

            else if (!gps && armed && stabilize)
                mavState = MAV_MODE_STABILIZE_ARMED;

            else if (!gps && !armed && stabilize)
                mavState = MAV_MODE_STABILIZE_DISARMED;

            mavlink_msg_heartbeat_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, uavtype, MAV_AUTOPILOT_GENERIC, mavState, 0, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);

            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            identSended = OK;

            break;

        case MSP_STATUS:
            // Send Status

            sensors |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present

            sensors |= MAV_SYS_STATUS_SENSOR_GPS;

            sensors |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

            // all present sensors enabled by default except altitude and position control which we will set individually
            sensors = sensors & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);

            mavlink_msg_sys_status_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, sensors, sensors, 0, (mwiState->cycleTime / 10), mwiState->vBat * 1000, mwiState->pAmp, mwiState->pMeterSum, mwiState->rssi, mwiState->i2cError, mwiState->debug[0], mwiState->debug[1],
//             mavlink_msg_sys_status_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, mwiState->sensors, mwiState->mode, 0, (mwiState->cycleTime / 10), mwiState->vBat * 1000, mwiState->pAmp, mwiState->pMeterSum, mwiState->rssi, mwiState->i2cError, mwiState->debug[0], mwiState->debug[1],

                    mwiState->debug[2], mwiState->debug[3]);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_RAW_IMU:
            // Send raw imu
            mavlink_msg_raw_imu_pack(mwiUavID, MAV_COMP_ID_IMU, &msg, currentTime, mwiState->ax, mwiState->ay, mwiState->az, mwiState->gx, mwiState->gy, mwiState->gz, mwiState->magx, mwiState->magy, mwiState->magz);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_SERVO:
            // Send servo - SERVO_CHAN
            mavlink_msg_servo_output_raw_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime, SERVO_CHAN, mwiState->servo[0], mwiState->servo[1], mwiState->servo[2], mwiState->servo[3], mwiState->servo[4], mwiState->servo[5], mwiState->servo[6], mwiState->servo[7]);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_MOTOR:
            // Send servo - MOTOR_CHAN
            mavlink_msg_servo_output_raw_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime, MOTOR_CHAN, mwiState->mot[0], mwiState->mot[1], mwiState->mot[2], mwiState->mot[3], mwiState->mot[4], mwiState->mot[5], mwiState->mot[6], mwiState->mot[7]);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_RC:
            // Send rcDate
            mavlink_msg_rc_channels_raw_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, currentTime / 1000, 1, mwiState->rcPitch, mwiState->rcRoll, mwiState->rcThrottle, mwiState->rcYaw, mwiState->rcAUX1, mwiState->rcAUX2, mwiState->rcAUX3, mwiState->rcAUX4, 255);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            // Send update hud
            mavlink_msg_vfr_hud_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, 0, 0, mwiState->head, (mwiState->rcThrottle - 1000) / 10, mwiState->baro / 100.0f, mwiState->vario / 100.0f);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_RAW_GPS:
            // Send gps
            mavlink_msg_gps_raw_int_pack(mwiUavID, MAV_COMP_ID_GPS, &msg, currentTime, mwiState->GPS_fix + 1, mwiState->GPS_latitude, mwiState->GPS_longitude, mwiState->GPS_altitude * 1000.0, 0, 0, mwiState->GPS_speed, 0, mwiState->GPS_numSat);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            mavlink_msg_global_position_int_pack(mwiUavID, MAV_COMP_ID_GPS, &msg, currentTime / 1000, mwiState->GPS_latitude, mwiState->GPS_longitude, mwiState->GPS_altitude * 10.0, mwiState->baro * 10, 0, 0, 0, 0);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

            break;

        case MSP_COMP_GPS:
            break;

        case MSP_ATTITUDE:
            // Send attitude
            mavlink_msg_attitude_pack(mwiUavID, MAV_COMP_ID_IMU, &msg, currentTime / 1000, deg2radian(mwiState->angx), -deg2radian(mwiState->angy), deg2radian(mwiState->head), deg2radian(mwiState->gx), deg2radian(mwiState->gy), deg2radian(mwiState->gz));
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, (char)len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);
            break;

        case MSP_ALTITUDE:
            // Send Local Position - unused without other sensors or gps cord
//            mavlink_msg_local_position_ned_pack(mwiUavID, 200, &msg, currentTime, 0, 0, 0, 0, 0, 0);
//            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
//            sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&groundStationAddr, size);
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
            mavlink_msg_debug_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, (uint32_t)(currentTime / 1000), 1, mwiState->serialErrorsCount);
            len = (char)mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);
            break;

        case MSP_BOXNAMES:
            break;

        case MSP_PIDNAMES:
            break;

        case NOK:
            break;
    }
}

// gc -> fc
void handleMessage(mavlink_message_t* currentMsg)
{
    switch (currentMsg->msgid) {

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            // decode
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

            printf("got MAVLINK_MSG_ID_PARAM_REQUEST_LIST\n");
            // decode
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(currentMsg, &packet);

            // Start sending parameters 
            int32_t i, p = 1;
            char name[8] = "PID_ _";

            for (i = 0; i < PIDITEMS; i++) {
                name[4] = i + '0';
                name[6] = 'P';
                mavlink_msg_param_value_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, name, (int32_t)mwiState->byteP[i], MAVLINK_TYPE_FLOAT, 3 * PIDITEMS, p++);
                len = (char)mavlink_msg_to_send_buffer(buf, &msg);
                sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

                name[4] = i + '0';
                name[6] = 'I';
                mavlink_msg_param_value_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, name, (int32_t)mwiState->byteI[i], MAVLINK_TYPE_FLOAT, 3 * PIDITEMS, p++);
                len = (char)mavlink_msg_to_send_buffer(buf, &msg);
                sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);

                name[4] = i + '0';
                name[6] = 'D';
                mavlink_msg_param_value_pack(mwiUavID, MAV_COMP_ID_ALL, &msg, name, (int32_t)mwiState->byteD[i], MAVLINK_TYPE_FLOAT, 3 * PIDITEMS, p++);
                len = (char)mavlink_msg_to_send_buffer(buf, &msg);
                sendto(sock, (const char *)buf, len, 0, (struct sockaddr*)&groundStationAddr, sizeGroundStationAddr);
            }

            break;
        }

        case MAVLINK_MSG_ID_PARAM_SET: {
            // decode
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

            printf("key = %s, value = %g\n", key, packet.param_value);

            //            // Report back new value
            //            char param_name[ONBOARD_PARAM_NAME_LENGTH];             // XXX HACK - need something to return a char *
            //            vp->copy_name(param_name, sizeof(param_name));
            //            mavlink_msg_param_value_send(chan,
            //            (int8_t*)param_name,
            //            packet.param_value,
            //            _count_parameters(),
            //            -1);                       // XXX we don't actually know what its index is...

            break;
        } // end case

    }
}

void eexit(int code)
{

#if defined( _WINDOZ)
    CloseHandle(serialLink);
    WSACleanup();
#else
    if (serialLink > 0) {
        close(serialLink);
    }
#endif

    if (sock > 0) {
        close(sock);
    }
    exit(code);
}

