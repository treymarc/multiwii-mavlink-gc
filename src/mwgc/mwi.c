#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#include <stdint.h>

#include "../mav/common/mavlink.h"
#include "../serial/serial.h"
#include "../include/utils.h"
#include "mwi.h"

int MASK =0xff;
#define IDLE  0
#define HEADER_START 1
#define HEADER_M  2
#define HEADER_ARROW  3
#define HEADER_SIZE  4
#define  HEADER_CMD 5

#define BLENGTH 256

char frame[BLENGTH]; // serial frame
int readindex = 0; // read position in the serial frame
int writeindex = 0; // write position in the serial frame

uint8_t stateMSP = IDLE;

// TODO display localy
int serialBuffErrorsCount;

// for reading byte from serial frame
int read32(void);
int16_t read16(void);
int8_t read8(void);

void save(int aByte);
void setState(int aState);

void decode( mwi_uav_state_t *mwiState);

int MWIserialbuffer_init(const char* serialport){
	return serialport_init(serialport, SERIAL_DEFAULT_BAUDRATE);
}

int MWIserialbuffer_askForFrame(HANDLE serialPort, uint8_t MSP_ID) {

	char msg[6];
	int hash = 0;
	int payloadz = 0;

	hash ^= payloadz;
	hash ^= MSP_ID;

	//strcpy(msg, GS_TO_FC);
	msg[0] = MSP_HEAD1;
	msg[1] = MSP_HEAD2;
	msg[2] = MSP_TO_GC;
	msg[3] = payloadz;
	msg[4] = MSP_ID;
	msg[5] = hash;

	if (serialport_write(serialPort, msg) == -1) {
		// fail to write command to serial port
		return NOK;
	}
	return OK;
}




// assemble a byte of the reply packet into "frame"
void save(int aByte) {
	if (writeindex < BLENGTH){
		frame[writeindex++] = aByte;
	}else{
		MW_TRACE(" whoops ")
	}
}

void setState(int aState) {
	stateMSP = aState;
}

int read32(void) {
	int t = frame[readindex++] & 0xff;
	t += frame[readindex++] << 8;
	t += frame[readindex] << 6;
	return t;

}

int16_t read16(void) {
	int16_t t = frame[readindex++] & 0xff;
	t += frame[readindex++] << 8;
	return t;
}

int8_t read8(void) {
	return (frame[readindex++] & 0xff);
}


void decode( mwi_uav_state_t *mwiState) {
	//MW_TRACE("decoded");

	readindex = 0;
	int recievedCmd = read8();

	int i = 0;

	switch (recievedCmd) {
	case MSP_IDENT:
		MW_TRACE("MSP_IDENT")
		mwiState->version = read8();
		mwiState->multiType = read8();
		break;

	case MSP_STATUS:
		MW_TRACE("MSP_STATUS")
		mwiState->cycleTime = read16();
		mwiState->i2cError = read16();
		mwiState->present = read16();
		mwiState->mode = read16();
//								MAV_MODE_STABILIZE_ARMED
		// if ((present&1) >0) {buttonAcc.setColorBackground(green_);} else {buttonAcc.setColorBackground(red_);tACC_ROLL.setState(false); tACC_PITCH.setState(false); tACC_Z.setState(false);}
		// if ((present&2) >0) {buttonBaro.setColorBackground(green_);} else {buttonBaro.setColorBackground(red_); tBARO.setState(false); }
		// if ((present&4) >0) {buttonMag.setColorBackground(green_);} else {buttonMag.setColorBackground(red_); tMAGX.setState(false); tMAGY.setState(false); tMAGZ.setState(false); }
		// if ((present&8) >0) {buttonGPS.setColorBackground(green_);} else {buttonGPS.setColorBackground(red_); tHEAD.setState(false);}
		// if ((present&16)>0) {buttonSonar.setColorBackground(green_);} else {buttonSonar.setColorBackground(red_);}
		// for(i=0;i<CHECKBOXITEMS;i++) {
		//   if ((mode&(1<<i))>0) buttonCheckbox[i].setColorBackground(green_); else buttonCheckbox[i].setColorBackground(red_);
		// }
		break;

	case MSP_RAW_IMU:
		MW_TRACE("MSP_RAW_IMU")
		mwiState->ax = read16();
		mwiState->ay = read16();
		mwiState->az = read16();
		mwiState->gx = read16() / 8;
		mwiState->gy = read16() / 8;
		mwiState->gz = read16() / 8;
		mwiState->magx = read16() / 3;
		mwiState->magy = read16() / 3;
		mwiState->magz = read16() / 3;
		break;

	case MSP_SERVO:
		MW_TRACE("MSP_SERVO")
		for (i = 0; i < 8; i++)
			mwiState->servo[i] = read16();
		break;

	case MSP_MOTOR:
		MW_TRACE("MSP_MOTOR")
		for (i = 0; i < 8; i++)
			mwiState->mot[i] = read16();
		break;

	case MSP_RC:
		MW_TRACE("MSP_RC")
		mwiState->rcRoll = read16();
		mwiState->rcPitch = read16();
		mwiState->rcYaw = read16();
		mwiState->rcThrottle = read16();
		mwiState->rcAUX1 = read16();
		mwiState->rcAUX2 = read16();
		mwiState->rcAUX3 = read16();
		mwiState->rcAUX4 = read16();

		break;

	case MSP_RAW_GPS:
		// GPS_fix = read8();
		// GPS_numSat = read8();
		// GPS_latitude = read32();
		// GPS_longitude = read32();
		// GPS_altitude = read16();
		// GPS_speed = read16();
		MW_TRACE("MSP_RAW_GPS")
		mwiState->GPS_fix = read8();
		mwiState->GPS_numSat = read8();
		// mwiState->GPS_latitude = read32();
		// mwiState->GPS_longitude = read32();
		// mwiState->GPS_altitude = read16();
		// mwiState->GPS_speed = read16();
		/* Send gps */
		break;

	case MSP_COMP_GPS:
		// GPS_distanceToHome = read16();
		// GPS_directionToHome = read16();
		// GPS_update = read8();
		MW_TRACE("MSP_COMP_GPS")
		mwiState->GPS_distanceToHome = read16();
		mwiState->GPS_directionToHome = read16();
		mwiState->GPS_update = read8();
		break;

	case MSP_ATTITUDE:
		MW_TRACE("MSP_ATTITUDE")
		mwiState->angx = read16() / 10;
		mwiState->angy = read16() / 10;
		mwiState->head = read16();

		break;

	case MSP_ALTITUDE:
		MW_TRACE("MSP_ALTITUDE")
		mwiState->baro = read32();

		break;

	case MSP_BAT: // TODO SEND
		MW_TRACE("MSP_BAT")
		mwiState->bytevbat = read8();
		mwiState->pMeterSum = read16();
		break;

	case MSP_RC_TUNING:
		MW_TRACE("MSP_RC_TUNING")
		mwiState->byteRC_RATE = read8();
		mwiState->byteRC_EXPO = read8();
		mwiState->byteRollPitchRate = read8();
		mwiState->byteYawRate = read8();
		mwiState->byteDynThrPID = read8();
		break;

	case MSP_ACC_CALIBRATION:
		MW_TRACE("MSP_ACC_CALIBRATION")
		break;

	case MSP_MAG_CALIBRATION:
		MW_TRACE("MSP_MAG_CALIBRATION")
		break;

	case MSP_PID:
		MW_TRACE("MSP_PID")
		for (i = 0; i < PIDITEMS; i++) {
			mwiState->byteP[i] = read8();
			mwiState->byteI[i] = read8();
			mwiState->byteD[i] = read8();
			//   confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);
			//   confP[i].setColorBackground(green_);
			//   confI[i].setColorBackground(green_);
			//   confD[i].setColorBackground(green_);
		}
		break;

	case MSP_BOX:
		MW_TRACE("MSP_BOX")
		break;

	case MSP_MISC: // TODO SEND
		MW_TRACE("MSP_MISC")
		break;

	case MSP_MOTOR_PINS: // TODO SEND
		MW_TRACE("MSP_MOTOR_PINS")
		break;

	case MSP_DEBUG:
		MW_TRACE("MSP_DEBUG")
		mwiState->debug1 = read16();
		mwiState->debug2 = read16();
		mwiState->debug3 = read16();
		mwiState->debug4 = read16();
		break;

	case MSP_BOXNAMES:
		MW_TRACE("MSP_BOXNAMES")
		strcpy(mwiState->boxnames, frame);
		break;

	case MSP_PIDNAMES:
		MW_TRACE("MSP_PIDNAMES")
		break;
	}
}



/**
 *
 */
int MWIserialbuffer_readNewFrames(HANDLE serialPort, mwi_uav_state_t *mwiState) {
	int cmd = NOK; // incoming commande
	uint8_t readbuffer[1]; //
	uint8_t checksum=0; //
	uint8_t dataSize=0; // size of the incoming payload


	while (serialport_readChar(serialPort, readbuffer)) {
		//MW_TRACE(input)
		//printf("%i",input[0]);
		switch (stateMSP) {
		default:
			// stateMSP is at an unknown value, but this cannot happen
			// unless somebody introduces a bug.
			// fall thru just in case.

		case IDLE:
			if (readbuffer[0] == MSP_HEAD1) {
				setState(HEADER_START);
			} else {
				setState(IDLE);
			//	mwiState->serialErrorsCount=1+mwiState->serialErrorsCount;
			}
			break;

		case HEADER_START:
			if (readbuffer[0] == MSP_HEAD2) {
				setState(HEADER_M);
			} else {
				setState(IDLE);
				//mwiState->serialErrorsCount=1+mwiState->serialErrorsCount;
			}
			break;

		case HEADER_M:
			if (readbuffer[0] == MSP_TO_FC) {
				setState(HEADER_ARROW);
			} else {
				setState(IDLE);
			//	mwiState->serialErrorsCount=1+mwiState->serialErrorsCount;
			}
			break;

		case HEADER_ARROW: // got arrow, expect dataSize now
			// This is the count of bytes which follow AFTER the command
			// byte which is next. +1 because we save() the cmd byte too, but
			// it excludes the checksum

			dataSize = readbuffer[0] + 1;

			// reset index variables for save()
			writeindex = 0;
			checksum = readbuffer[0]; // same as: checksum = 0, checksum ^= input[0];

			// the command is to follow
			setState(HEADER_SIZE);
			break;

		case HEADER_SIZE: // got size, expect cmd now

			checksum ^= readbuffer[0];
			cmd = readbuffer[0];
			// pass the command byte to the ByteBuffer handler also
			save(readbuffer[0]);
			setState(HEADER_CMD);
			break;

		case HEADER_CMD: // got cmd, expect payload, if any, then checksum
			if (writeindex < dataSize) {
				// keep reading the payload in this state until offset==dataSize
				checksum ^= readbuffer[0];
				save(readbuffer[0]);

				// stay in this state
			} else {
				// done reading, reset the decoder for next byte
				setState(IDLE);

				if ((checksum & MASK) != readbuffer[0]) {
					MW_TRACE("checksum failed")
					//mwiState->serialErrorsCount=1+mwiState->serialErrorsCount;
				} else {
					decode(mwiState);
					return cmd;
				}
			}
			break;
		}
	}
	return NOK;
//	MW_TRACE(" end ")

}
