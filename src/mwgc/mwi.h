/*******************************************************************************
 Copyright (C) 2012  Trey Marc ( a t ) gmail.com

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

 ****************************************************************************/
#include "../serial/serial.h"

#ifndef MWI_NS_H
#define MWI_NS_H

#define  CHECKBOXITEMS 11
#define  PIDITEMS 8

// mwi data
typedef struct {

	char boxnames[256];
	int present, mode;
	int mot[8];
	int servo[8];

	int version;
	int gx, gy, gz, ax, ay, az, magx, magy, magz, baro, head, angx, angy,
			debug1, debug2, debug3, debug4;
	int GPS_distanceToHome, GPS_directionToHome;
	int GPS_numSat, GPS_fix, GPS_update;
	int time1, time2;
	int cycleTime, i2cError;
	int nunchukPresent, i2cAccPresent, i2cBaroPresent, i2cMagnetoPresent,
			GPSPresent, levelMode;
	int multiType; // 1 for tricopter, 2 for quad+, 3 for quadX, ...
	int pMeterSum;
	int PowerTrigger;
	int bytevbat;

	int rcThrottle, rcRoll, rcPitch, rcYaw, rcAUX1, rcAUX2, rcAUX3, rcAUX4;

	int byteP[PIDITEMS], byteI[PIDITEMS], byteD[PIDITEMS];
	int byteRC_RATE, byteRC_EXPO, byteRollPitchRate, byteYawRate, byteDynThrPID;
	int activation1[CHECKBOXITEMS], activation2[CHECKBOXITEMS];

	int serialErrorsCount;
} mwi_uav_state_t;




#define MWI_FULLFRAME_SIZE 64

#define   MSP_IDENT                100
#define   MSP_STATUS               101
#define   MSP_RAW_IMU              102
#define   MSP_SERVO                103
#define   MSP_MOTOR                104
#define   MSP_RC                   105
#define   MSP_RAW_GPS              106
#define   MSP_COMP_GPS             107
#define   MSP_ATTITUDE             108
#define   MSP_ALTITUDE             109
#define   MSP_BAT                  110
#define   MSP_RC_TUNING            111
#define   MSP_PID                  112
#define   MSP_BOX                  113
#define   MSP_MISC                 114
#define   MSP_MOTOR_PINS           115
#define   MSP_BOXNAMES             116
#define   MSP_PIDNAMES             117

#define   MSP_SET_RAW_RC           200
#define   MSP_SET_RAW_GPS          201
#define   MSP_SET_PID              202
#define   MSP_SET_BOX              203
#define   MSP_SET_RC_TUNING        204
#define   MSP_ACC_CALIBRATION      205
#define   MSP_MAG_CALIBRATION      206
#define   MSP_SET_MISC             207
#define   MSP_RESET_CONF           208

#define   MSP_EEPROM_WRITE         250

#define   MSP_DEBUG                254

// fligh controler to ground station
#define FC_TO_GS "$M>"

// ground station to fligh controler
#define GS_TO_FC "$M<"

#define MSP_HEAD1 '$'
#define MSP_HEAD2 'M'
#define MSP_TO_FC '>'
#define MSP_TO_GC '<'


// serial impl mwi.c
// mwi binary protocol
int MWIserialbuffer_askForFrame(HANDLE serialPort, uint8_t MSP_ID);
int MWIserialbuffer_readNewFrames(HANDLE serialPort, mwi_uav_state_t *mwiState);
int MWIserialbuffer_init(const char* serialport);

#endif

