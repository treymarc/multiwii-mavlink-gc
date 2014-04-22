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
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include "../include/utils.h"
#include "mwi.h"

int MASK = 0xff;
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

// for reading byte from serial frame
int32_t read32(void);
int16_t read16(void);
int8_t read8(void);

void save(int aByte);
void setState(int aState);

void decode(mwi_uav_state_t *mwiState);

HANDLE MWIserialbuffer_init(const char* serialport, int baudrate)
{
    return serialport_init(serialport, baudrate);
}

int MWIserialbuffer_askForFrame(HANDLE serialPort, uint8_t MSP_ID)
{
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
void save(int aByte)
{
    if (writeindex < BLENGTH) {
        frame[writeindex++] = aByte;
    } else {
        MW_TRACE(" serial buffer overrun \n ")
    }
}

void setState(int aState)
{
    stateMSP = aState;
}

int read32(void)
{
    int32_t t = frame[readindex++] & 0xff;
    t += (frame[readindex++] & 0xff) << 8;
    t += (frame[readindex++] & 0xff) << 16;
    t += (frame[readindex++] & 0xff) << 24;
    return t;

}

int16_t read16(void)
{
    int16_t t = frame[readindex++] & 0xff;
    t += frame[readindex++] << 8;
    return t;
}

uint16_t readu16(void)
{
    uint16_t t = (uint16_t)frame[readindex++] & 0xff;
    t += (uint16_t)frame[readindex++] << 8;
    return t;
}


int8_t read8(void)
{
    return (frame[readindex++] & 0xff);
}

void decode(mwi_uav_state_t *mwiState)
{
    //MW_TRACE("decoded");
    readindex = 0;
    uint8_t recievedCmd = read8();

    int i = 0;

    switch (recievedCmd) {
        case MSP_IDENT:
            MW_TRACE("MSP_IDENT\n")
            mwiState->version = read8();
            mwiState->multiType = read8();
            mwiState->mspVersion = read8();
            mwiState->capability = read32();
            break;

        case MSP_STATUS:
            MW_TRACE("MSP_STATUS\n")
            mwiState->cycleTime = read16();
            mwiState->i2cError = read16();
            mwiState->sensors = read16();
            mwiState->mode = read32();
            mwiState->profile = read8();

            for (i = 0; i < mwiState->boxcount; i++) {
                (mwiState->box[i])->state = ((mwiState->mode & (1 << i)) > 0);
                //printf("%s = %d",(mwiState->box[i])->name,(mwiState->box[i])->state);
            }
            break;

        case MSP_RAW_IMU:
            MW_TRACE("MSP_RAW_IMU\n")
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
            MW_TRACE("MSP_SERVO\n")
            for (i = 0; i < 8; i++)
                mwiState->servo[i] = read16();
            break;

        case MSP_MOTOR:
            MW_TRACE("MSP_MOTOR\n")
            for (i = 0; i < 8; i++)
                mwiState->mot[i] = read16();
            break;

        case MSP_RC:
            MW_TRACE("MSP_RC\n")
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
            MW_TRACE("MSP_RAW_GPS\n")
            mwiState->GPS_fix = read8();
            mwiState->GPS_numSat = read8();
            mwiState->GPS_latitude = read32();
            mwiState->GPS_longitude = read32();
            mwiState->GPS_altitude = read16();
            mwiState->GPS_speed = (uint16_t)readu16();
            /* Send gps */
            break;

        case MSP_COMP_GPS:
            MW_TRACE("MSP_COMP_GPS\n")
            mwiState->GPS_distanceToHome = read16();
            mwiState->GPS_directionToHome = read16();
            mwiState->GPS_update = read8();
            break;

        case MSP_ATTITUDE:
            MW_TRACE("MSP_ATTITUDE\n")
            mwiState->angx = read16() / 10;
            mwiState->angy = read16() / 10;
            mwiState->head = read16();
            break;

        case MSP_ALTITUDE:
            MW_TRACE("MSP_ALTITUDE\n")
            mwiState->baro = read32();
            mwiState->vario = read16();
            break;

        case MSP_ANALOG: // TODO SEND
            MW_TRACE("MSP_BAT\n")
            mwiState->bytevbat = read8();
            mwiState->pMeterSum = read16();
            mwiState->rssi = read16();
            mwiState->pAmp = read16();
            break;

        case MSP_RC_TUNING:
            MW_TRACE("MSP_RC_TUNING\n")
            mwiState->byteRC_RATE = read8();
            mwiState->byteRC_EXPO = read8();
            mwiState->byteRollPitchRate = read8();
            mwiState->byteYawRate = read8();
            mwiState->byteDynThrPID = read8();
            mwiState->byteRC_thrMid = read8();
            mwiState->byteRC_thrExpo = read8();
            break;

        case MSP_ACC_CALIBRATION:
            MW_TRACE("MSP_ACC_CALIBRATION\n")
            break;

        case MSP_MAG_CALIBRATION:
            MW_TRACE("MSP_MAG_CALIBRATION\n")
            break;

        case MSP_PID:
            MW_TRACE("MSP_PID\n")
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
            MW_TRACE("MSP_BOX\n")
            break;

        case MSP_MISC: // TODO SEND
            MW_TRACE("MSP_MISC\n")
            break;

        case MSP_MOTOR_PINS: // TODO SEND
            MW_TRACE("MSP_MOTOR_PINS\n")
            break;

        case MSP_DEBUG:
            MW_TRACE("MSP_DEBUG\n")
            for (i = 0; i < DEBUGITEMS; i++) {
                mwiState->debug[i] = read16();
            }
            break;

        case MSP_BOXNAMES:
            MW_TRACE("MSP_BOXNAMES\n")
            char boxnames[256] = "";
            strcpy(boxnames, frame);
            char* boxname = NULL;
            if (strlen(boxnames) < 1)
                return;

            memmove(boxnames, boxnames+1, strlen(boxnames));
            boxname = strtok(boxnames, ";");
            mwiState->boxcount = 0;

            while (boxname != NULL) {
                if (mwiState->box[mwiState->boxcount] != 0){
                    free(mwiState->box[mwiState->boxcount]);
                }
                mwiState->box[mwiState->boxcount] = malloc(sizeof(*mwiState->box[mwiState->boxcount]));

                strcpy(mwiState->box[mwiState->boxcount]->name, boxname);

                MW_TRACE(boxname)
                MW_TRACE("\n")
                mwiState->boxcount += 1;
                boxname = strtok(NULL, ";");
            }
            break;

        case MSP_PIDNAMES:
            MW_TRACE("MSP_PIDNAMES\n")
            break;
        case MSP_PRIVATE:
            MW_TRACE("MSP_PRIVATE\n")
            break;
    }
    mwiState->callback(recievedCmd);
}

void MWIserialbuffer_readNewFrames(HANDLE serialPort, mwi_uav_state_t *mwiState)
{
    uint8_t readbuffer[1]; //
    uint8_t checksum = 0; //
    uint8_t dataSize = 0; // size of the incoming payload

    while (serialport_readChar(serialPort, (char *)readbuffer)) {

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
                        MW_TRACE("msp checksum failed\n")
                        mwiState->serialErrorsCount += 1;
                    } else {
                        decode(mwiState);
                    }
                }
                break;
        }
    }
//	MW_TRACE(" end ")

}

/* QNX timer version */
#if (defined __QNX__) || (defined __QNXNTO__)
uint64_t microsSinceEpoch(void) {
    struct timespec time;
    uint64_t micros = 0;

    clock_gettime(CLOCK_REALTIME, &time);
    micros = (uint64_t)time.tv_sec * 100000 + time.tv_nsec/1000;

    return micros;
}
#else
uint64_t microsSinceEpoch(void)
{
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);
    micros = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}
#endif
