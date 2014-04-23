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
#include <stdint.h>
#include "../serial/serial.h"

#ifndef MWI_NS_H
#define MWI_NS_H

#define CHECKBOXITEMS 11
#define PIDITEMS 8
#define DEBUGITEMS 4
// mwi data
typedef struct {
    char name[32];
    int state;
} mwi_box_t;

typedef struct mwi_uav_state_t {
    mwi_box_t* box[32];
    int version, sensors, mode, profile, boxcount;
    int capability, mspVersion;
    int mot[8];
    int servo[8];
    int debug[4];

    int gx, gy, gz, ax, ay, az, magx, magy, magz, baro, vario, head, angx, angy;

    int GPS_distanceToHome, GPS_directionToHome;
    int GPS_numSat, GPS_fix, GPS_update, GPS_speed, GPS_heading;
    uint16_t GPS_altitude;
    int GPS_latitude, GPS_longitude;
    int time1, time2;
    int cycleTime, i2cError;
    int nunchukPresent, i2cAccPresent, i2cBaroPresent, i2cMagnetoPresent, GPSPresent, levelMode;
    int multiType; // 1 for tricopter, 2 for quad+, 3 for quadX, ...
    int pMeterSum, pAmp;
    int PowerTrigger;
    int vBat;

    int rssi;

    int rcThrottle, rcRoll, rcPitch, rcYaw, rcAUX1, rcAUX2, rcAUX3, rcAUX4;

    int byteP[PIDITEMS], byteI[PIDITEMS], byteD[PIDITEMS];
    int byteRC_RATE, byteRC_EXPO, byteRollPitchRate, byteRC_thrMid, byteRC_thrExpo, byteYawRate, byteDynThrPID;
    int activation1[CHECKBOXITEMS], activation2[CHECKBOXITEMS];

    int serialErrorsCount;

    void (*callback)(int);
} mwi_uav_state_t;

#define   MWI_FULLFRAME_SIZE 64

#define   MSP_PRIVATE                1

#define   MSP_IDENT                100    //out message         multitype + multiwii version + protocol version + capability variable
#define   MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define   MSP_RAW_IMU              102    //out message         9 DOF
#define   MSP_SERVO                103    //out message         8 servos
#define   MSP_MOTOR                104    //out message         8 motors
#define   MSP_RC                   105    //out message         8 rc chan and more
#define   MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define   MSP_COMP_GPS             107    //out message         distance home, direction home
#define   MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define   MSP_ALTITUDE             109    //out message         altitude, variometer
#define   MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define   MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define   MSP_PID                  112    //out message         P I D coeff (9 are used currently)
#define   MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
#define   MSP_MISC                 114    //out message         powermeter trig
#define   MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define   MSP_BOXNAMES             116    //out message         the aux switch names
#define   MSP_PIDNAMES             117    //out message         the PID names
#define   MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define   MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define   MSP_SERVO_CONF           120    //out message         Servo settings
#define   MSP_NAV_STATUS           121    //out message         Returns navigation status
#define   MSP_NAV_CONFIG           122    //out message         Returns navigation parameters

#define   MSP_SET_RAW_RC           200
#define   MSP_SET_RAW_GPS          201
#define   MSP_SET_PID              202
#define   MSP_SET_BOX              203
#define   MSP_SET_RC_TUNING        204
#define   MSP_ACC_CALIBRATION      205
#define   MSP_MAG_CALIBRATION      206
#define   MSP_SET_MISC             207
#define   MSP_RESET_CONF           208
#define   MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define   MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define   MSP_SET_HEAD             211    //in message          define a new heading hold direction
#define   MSP_SET_SERVO_CONF       212    //in message          Servo settings
#define   MSP_SET_MOTOR            214    //in message          PropBalance function
#define   MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom

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
#define MSP_ERROR '!'

// serial impl mwi.c
// mwi binary protocol
int MWIserialbuffer_askForFrame(HANDLE serialPort, uint8_t MSP_ID);
void MWIserialbuffer_readNewFrames(HANDLE serialPort, mwi_uav_state_t *mwiState);
HANDLE MWIserialbuffer_init(const char* serialport, int baudrate);
uint64_t microsSinceEpoch(void);

#endif

