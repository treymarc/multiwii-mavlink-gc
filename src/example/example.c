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

#include "../mwgc/mwi.h"
#include "../include/utils.h"

//default value
#define DEFAULT_SERIAL_DEV "/dev/ttyO2"

// global : serialPort
HANDLE serialLink;


int main(int argc, char* argv[]) {

#if defined( _WINDOZ )
	WSADATA WSAData;
	WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

	// serail devices "COME5" or /dev/ttyUSB0 ..
	char serialDevice[150];

	strcpy(serialDevice, DEFAULT_SERIAL_DEV);
	for (int i = 1; i < argc; i++) {
		if (i + 1 != argc) {
			if (strcmp(argv[i], "-s") == 0) {
				strcpy(serialDevice, argv[i + 1]);
				i++;
			}
		}

	}

	MW_TRACE("starting..\n")
	serialLink = 1;
	serialLink = MWIserialbuffer_init(serialDevice);

	if (serialLink <= 0) {
		perror("error open serial");
		return -1;
	}

	// mwi state -
	mwi_uav_state_t *mwiState;
	mwiState = malloc(sizeof(*mwiState));


	int initOk = NOK;
	int state = 0;

	for (;;) {

		if (initOk == OK) {

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

		} else {
			MWIserialbuffer_askForFrame(serialLink, MSP_IDENT);
		}

		state = MWIserialbuffer_readNewFrames(serialLink, mwiState);

		printf(" MSP : %i", state);

		switch (state) {
		case MSP_IDENT:
			initOk = OK;
			break;

		case MSP_STATUS:
			break;

		case MSP_RAW_IMU:
			break;

		case MSP_SERVO:
			break;

		case MSP_MOTOR:
			break;

		case MSP_RC:
			break;

		case MSP_RAW_GPS:
			break;

		case MSP_COMP_GPS:
			break;

		case MSP_ATTITUDE:
			break;

		case MSP_ALTITUDE:
			break;

		case MSP_BAT:
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

		case MSP_MISC:
			break;

		case MSP_MOTOR_PINS:
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

		usleep(5000);

	}
}
