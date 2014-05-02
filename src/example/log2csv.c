/*******************************************************************************
 Copyright (C) 2013  Trey Marc ( a t ) gmail.com

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

 -2013.12.18 : demo code

 ****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "../utils/utils.h"
#include "../mwi/mwi.h"

HANDLE serialLink = NOK;
int initOk = NOK;
mwi_mav_t *mwiState;
uint64_t currentTime;

void callBack_mwi(int state);

int main(int argc, char* argv[])
{
    // serial devices "COM5" or /dev/ttyUSB0 ..
    char serialDevice[256] = "";

    for (int i = 1; i < argc; i++) {
        if (i + 1 != argc) {
            if (strcmp(argv[i], "-s") == 0) {
                strcpy(serialDevice, argv[i + 1]);
                i++;
            }
        }
    }

    MW_TRACE("starting..\n")
    serialLink = MWIserialbuffer_init(serialDevice, SERIAL_115200_BAUDRATE);

    if (serialLink == NOK) {
        perror("error opening serial port");
        return EXIT_FAILURE;
    }

    // mwi state
    mwiState = calloc(sizeof(*mwiState), sizeof(*mwiState));
    mwiState->callback = &callBack_mwi;

    uint64_t lastFrameRequest = 0;

    printf("currentTime;angx;angy;head;ax;ay;az;gx;gy;gz;magx;magy;magz;lat;lon;alt;numsat;mot[0];mot[1];mot[2];mot[3];mot[4];mot[5];rcRoll;rcPitch;rcYaw;rcThrottle;rcAUX1;rcAUX2;rcAUX3;rcAUX4;debug1;debug2;debug3;debug4\n");

    msp_payload_t *payload;
    payload = calloc(sizeof(*payload), sizeof(*payload));
    for (;;) {

        currentTime = microsSinceEpoch();

        if ((currentTime - lastFrameRequest) > 1000 * 30) {
            if (initOk == OK) {
                lastFrameRequest = currentTime;
                MWIserialbuffer_askForFrame(serialLink, MSP_RAW_IMU, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_DEBUG, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_ANALOG, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_ALTITUDE, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_COMP_GPS, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RAW_GPS, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RC, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_MOTOR, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_SERVO, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_RAW_IMU, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_STATUS, payload);
                MWIserialbuffer_askForFrame(serialLink, MSP_ATTITUDE, payload);
            } else {
                MWIserialbuffer_askForFrame(serialLink, MSP_IDENT, payload);
            }
        }

        MWIserialbuffer_readNewFrames(serialLink, mwiState);

        usleep(5000);
    }
}

void callBack_mwi(int state)
{
    switch (state) {
        case MSP_IDENT:
            initOk = OK;
            break;
        default:
            currentTime = microsSinceEpoch();
            printf("%ju;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i\n", currentTime, mwiState->angx, mwiState->angy, mwiState->head, mwiState->ax, mwiState->ay, mwiState->az, mwiState->gx, mwiState->gy, mwiState->gz,
                    mwiState->magx, mwiState->magy, mwiState->magz, mwiState->GPS_latitude, mwiState->GPS_longitude, mwiState->GPS_altitude, mwiState->GPS_numSat, mwiState->mot[0], mwiState->mot[1], mwiState->mot[2], mwiState->mot[3], mwiState->mot[4], mwiState->mot[5],
                    mwiState->rcRoll, mwiState->rcPitch, mwiState->rcYaw, mwiState->rcThrottle, mwiState->rcAUX1, mwiState->rcAUX2, mwiState->rcAUX3, mwiState->rcAUX4, mwiState->debug[0], mwiState->debug[1], mwiState->debug[2], mwiState->debug[3]);
            break;
    }

}
