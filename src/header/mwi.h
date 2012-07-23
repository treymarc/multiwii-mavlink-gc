
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

#ifndef MWI_H
#define MWI_H



#define  CHECKBOXITEMS 11
#define  PIDITEMS 8


// mwi data
typedef struct {

	int present , mode ;
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

	int rcThrottle , rcRoll , rcPitch , rcYaw , rcAUX1 , rcAUX2 , rcAUX3 , rcAUX4 ;

	int byteP[PIDITEMS], byteI[PIDITEMS], byteD[PIDITEMS];
	int byteRC_RATE, byteRC_EXPO, byteRollPitchRate, byteYawRate, byteDynThrPID;
	int activation1[CHECKBOXITEMS], activation2[CHECKBOXITEMS];

	int serialErrorsCount;
} mwi_uav_state_t;




#endif
