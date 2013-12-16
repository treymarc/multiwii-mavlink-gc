/*******************************************************************************
 derivative work from :

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

 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* timer headers */
#include <sys/time.h>
#include <time.h>

/* windows headers */
#if defined( _WINDOZ )

#include <winsock.h>
#define SocketErrno (WSAGetLastError())
#define bcopy(src,dest,len) memmove(dest,src,len)
typedef uint32_t socklen_t;

#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <netdb.h> /* gethostbyname */
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

#endif

#endif

/* mavlink message headers*/
#include "../mav/common/mavlink.h"

#include "mwi.h"
#include "../include/utils.h"

void eexit(int code);
void rtfmHelp(void);
void rtfmArgvErr(char* argv);
void rtfmVersion(void);

uint64_t microsSinceEpoch(void);

// handle incoming udp mavlink msg
void handleMessage(mavlink_message_t* currentMsg);

//default value
#define DEFAULT_UAVID 1
#define DEFAULT_IP_GS "127.0.0.1"
#define DEFAULT_SERIAL_DEV "/dev/ttyO2"

// global : serialPort, socket , uavId, groudnstation
HANDLE serialLink=0;
SOCKET sock;
short mwiUavID;
struct sockaddr_in groundStationAddr;

int main(int argc, char* argv[]) {

#if defined( _WINDOZ )
	WSADATA WSAData;
	WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

	char targetIp[150];
	char serialDevice[150];

	mwiUavID = DEFAULT_UAVID;
	strcpy(serialDevice, DEFAULT_SERIAL_DEV);
	strcpy(targetIp, DEFAULT_IP_GS);

	//	TODO set serialBaudRate for slow serial link;

	uint8_t udpInBuf[BUFFER_LENGTH];
	struct sockaddr_in locAddr;

	ssize_t recsize;
	socklen_t fromlen;

	// Check if --help flag was used
	if ((argc == 2) && (strcmp(argv[1], "--help") == 0)) {
		rtfmHelp();
	}

	// Check if --version flag was used
	if ((argc == 2) && (strcmp(argv[1], "--version") == 0)) {
		rtfmVersion();
	}

	// parse flag : -ip , -s , -id
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
			}
		}

	}

	printf("ground station ip: %s\n", targetIp);
	printf("serial link: %s\n", serialDevice);
	printf("uav id: %i\n", mwiUavID);

	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
	if (NOK
			== bind(sock, (struct sockaddr *) &locAddr,
					sizeof(struct sockaddr))) {
		perror("error bind failed");
		eexit(EXIT_FAILURE);
	}

	/* Attempt to make it non blocking */
#if defined(_WINDOZ)

	int arg = 1;
	if(ioctlsocket(sock, FIONBIO, &arg )<0) {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		eexit(EXIT_FAILURE);
	}

#else

	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		eexit(EXIT_FAILURE);
	}

#endif

	memset(&groundStationAddr, 0, sizeof(groundStationAddr));
	groundStationAddr.sin_family = AF_INET;
	groundStationAddr.sin_addr.s_addr = inet_addr(targetIp);
	groundStationAddr.sin_port = htons(14550);

	serialLink = MWIserialbuffer_init(serialDevice);

	if (serialLink <= 0) {
		perror("error open serial");
		return -1;
	}

	// mwi state -
	mwi_uav_state_t *mwiState;
	mwiState = malloc(sizeof(*mwiState));
	mwiState->mode = MAV_MODE_PREFLIGHT; // initial mode is unknow

	MW_TRACE("starting..\n")

	uint64_t lastFrameRequest = 0;
	uint64_t lastHeartBeat = 0;
	uint64_t currentTime = microsSinceEpoch();
	int state = NOK;
	int identSended = NOK;
	for (;;) {
		//MW_TRACE("beginloop\n")

		currentTime = microsSinceEpoch();

		if ((currentTime - lastFrameRequest) > 1000 * 30) {

			if (identSended == OK) {
				lastFrameRequest = currentTime;
				MWIserialbuffer_askForFrame(serialLink, MSP_RAW_IMU);
				MWIserialbuffer_askForFrame(serialLink, MSP_DEBUG);
				MWIserialbuffer_askForFrame(serialLink, MSP_BAT);
				MWIserialbuffer_askForFrame(serialLink, MSP_ALTITUDE);
				MWIserialbuffer_askForFrame(serialLink, MSP_COMP_GPS);
				MWIserialbuffer_askForFrame(serialLink, MSP_RAW_GPS);
				MWIserialbuffer_askForFrame(serialLink, MSP_RC);
				MWIserialbuffer_askForFrame(serialLink, MSP_MOTOR);
				MWIserialbuffer_askForFrame(serialLink, MSP_SERVO);
				MWIserialbuffer_askForFrame(serialLink, MSP_RAW_IMU);
				MWIserialbuffer_askForFrame(serialLink, MSP_STATUS);
				MWIserialbuffer_askForFrame(serialLink, MSP_ATTITUDE);

				if ((currentTime - lastHeartBeat) > 1000 * 500) {
					lastHeartBeat = currentTime;
					MWIserialbuffer_askForFrame(serialLink, MSP_IDENT);
				}

			} else {
				MWIserialbuffer_askForFrame(serialLink, MSP_IDENT);
			}
			//TODO
			//MSP_MOTOR
			//MSP_COMP_GPS
			//MSP_BAT
			//MSP_DEBUG

		}

		// mavlink msg
		int len = 0;
		uint8_t buf[BUFFER_LENGTH];
		mavlink_message_t msg;

		{
			state = MWIserialbuffer_readNewFrames(serialLink, mwiState);
			printf("MSP : %i", state);

			switch (state) {
			case MSP_IDENT:

				mavlink_msg_heartbeat_pack(mwiUavID, 200, &msg,
						MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
						mwiState->mode, 0, MAV_STATE_ACTIVE);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				identSended = OK;

				break;

			case MSP_STATUS:

				/* Send Status */
				mavlink_msg_sys_status_pack(mwiUavID, 200, &msg,
						mwiState->present, mwiState->mode, 0,
						(mwiState->cycleTime / 10), mwiState->bytevbat * 1000,
						mwiState->pMeterSum, -1, 0, mwiState->serialErrorsCount,
						mwiState->i2cError, mwiState->debug2, mwiState->debug3,
						mwiState->debug4);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_RAW_IMU:

				/* Send raw imu */

				mavlink_msg_raw_imu_pack(mwiUavID, 200, &msg, currentTime,
						mwiState->ax, mwiState->ay, mwiState->az, mwiState->gx,
						mwiState->gy, mwiState->gz, mwiState->magx,
						mwiState->magy, mwiState->magz);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_SERVO:
				/* Send servo */
				mavlink_msg_servo_output_raw_pack(mwiUavID, 200, &msg,
						currentTime, 1, mwiState->servo[0], mwiState->servo[1],
						mwiState->servo[2], mwiState->servo[3],
						mwiState->servo[4], mwiState->servo[5],
						mwiState->servo[6], mwiState->servo[7]);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_MOTOR:
				mavlink_msg_servo_output_raw_pack(mwiUavID, 200, &msg,
						currentTime, 2, mwiState->mot[0], mwiState->mot[1],
						mwiState->mot[2], mwiState->mot[3], mwiState->mot[4],
						mwiState->mot[5], mwiState->mot[6], mwiState->mot[7]);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_RC:
				/* Send rcDate */
				mavlink_msg_rc_channels_raw_pack(mwiUavID, 200, &msg,
						currentTime, 1, mwiState->rcPitch, mwiState->rcRoll,
						mwiState->rcThrottle, mwiState->rcYaw, mwiState->rcAUX1,
						mwiState->rcAUX2, mwiState->rcAUX3, mwiState->rcAUX4,
						255);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_RAW_GPS:
				/* Send gps */
				mavlink_msg_gps_raw_int_pack(mwiUavID, 200, &msg, currentTime,
						mwiState->GPS_fix + 1, 48.0, 7.0, 100.0, 0, 0, 0, 0, 0);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));

				break;

			case MSP_COMP_GPS:
				break;

			case MSP_ATTITUDE:
				/* Send attitude */
				mavlink_msg_attitude_pack(mwiUavID, 200, &msg, currentTime,
				deg2radian(mwiState->angx), -deg2radian(mwiState->angy),
				deg2radian(mwiState->head),
				deg2radian(mwiState->gx),
				deg2radian(mwiState->gy),
				deg2radian(mwiState->gz));
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));
				break;

			case MSP_ALTITUDE:
				/* Send Local Position */
				mavlink_msg_local_position_ned_pack(mwiUavID, 200, &msg,
						currentTime, 0, 0, 0, 0, 0, 0);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(sock, buf, len, 0, (struct sockaddr*) &groundStationAddr,
						sizeof(struct sockaddr_in));
				break;

			case MSP_BAT: // TODO SEND
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
				break;

			case MSP_BOXNAMES:
				break;

			case MSP_PIDNAMES:

				break;
			case NOK:

				break;
			}
		}

		//  udp in
		memset(udpInBuf, 0, BUFFER_LENGTH);
		//MW_TRACE("udp listen ? ")

		recsize = recvfrom(sock, (void *) udpInBuf, BUFFER_LENGTH, 0,
				(struct sockaddr *) &groundStationAddr, &fromlen);
		//MW_TRACE("..")
		if (recsize > 0) {
			MW_TRACE("\n")MW_TRACE(" <-- udp in <--\n")
			// Something received - print out all bytes and parse packet
			mavlink_message_t msgIn;
			mavlink_status_t status;
			unsigned int temp = 0;
			printf("Bytes Received: %d\nDatagram: ", (int) recsize);
			for (int i = 0; i < recsize; ++i) {
				temp = udpInBuf[i];
				printf("%02x ", (unsigned char) temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, udpInBuf[i], &msgIn,
						&status)) {
					// Packet received
					printf(
							"\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n",
							msgIn.sysid, msgIn.compid, msgIn.len, msgIn.msgid);
				}
			}
			printf("\n");
		}MW_TRACE("\n")
		// end udp in

		// no need to rush
		usleep(5000);

		//MW_TRACE("endloop\n")

	}
}

//--
void handleMessage(mavlink_message_t* currentMsg) {
	switch (currentMsg->msgid) {

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
		// decode
		mavlink_request_data_stream_t packet;
		mavlink_msg_request_data_stream_decode(currentMsg, &packet);

		int freq = 0; // packet frequency

		if (packet.start_stop == 0)
			freq = 0; // stop sending
		else if (packet.start_stop == 1)
			freq = packet.req_message_rate; // start sending
		else
			break;

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

		break;
	}

	case MAVLINK_MSG_ID_PARAM_SET: {
		// decode
		mavlink_param_set_t packet;
		mavlink_msg_param_set_decode(currentMsg, &packet);

		// set parameter
		const char * key = (const char*) packet.param_id;

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

		printf("key = %d, value = %g\n", key, packet.param_value);

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
//--

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{

	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_REALTIME, &time);
	micros = (uint64_t)time.tv_sec * 100000 + time.tv_nsec/1000;

	return micros;
}
#else
uint64_t microsSinceEpoch() {

	struct timeval tv;

	uint64_t micros = 0;

	gettimeofday(&tv, NULL );
	micros = ((uint64_t) tv.tv_sec) * 1000000 + tv.tv_usec;

	return micros;
}
#endif

void eexit(int code) {

#if defined( _WINDOZ)
	CloseHandle(serialLink);
	WSACleanup();
#else
	if (serialLink > 0) {
		close(serialLink);
	}
#endif

	if (sock != NOK) {
		close(sock);
	}
	exit(code);
}

void rtfmVersion(void) {
	printf("\n\nVersion: ");
	printf(MWGC_VERSION);
	printf("\n\n");
	eexit(EXIT_SUCCESS);

}

void rtfmHelp(void) {
	printf("\n");
	printf("\n\nmwgc - serial 2 udp");
	printf("\n");
	printf("\nUsage:\n\n");

	printf("\t -ip <ip address of QGroundControl>\n");
	printf("\t  default value : 127.0.0.1\n\n");
	printf("\t -s <serial device name>\n");
	printf("\t  default value : /dev/ttyO2\n\n");
	printf("\t -id <id for mavlink>\n");
	printf("\t  default value : 1\n\n");

	printf("\n");
	printf("\nOptions:\n\n");

	printf("\t--help");
	printf("\t  display this message\n\n");
	printf("\t --version");
	printf("\t  show version number\n\n");

	eexit(EXIT_SUCCESS);
}

void rtfmArgvErr(char* argv) {
	printf("\n");
	printf("\nInvalides arguments : %s\n", argv);
	printf("\n\nUsages : mwgc --help");
	printf("\n\n");

}
