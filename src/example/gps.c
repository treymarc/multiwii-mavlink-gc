/*******************************************************************************
 Copyright (C) 2014  Trey Marc ( a t ) gmail.com

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

     -2014.04.23 : payload demo set raw gps values
         TODO , write16:32 helper

 ****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "../include/utils.h"
#include "../mwi/mwi.h"

HANDLE serialLink = NOK;
int initOk = NOK;

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
        exit(EXIT_FAILURE);
    }

    // mwi state
    mwi_uav_state_t *mwiState;
    mwiState = calloc(sizeof(*mwiState), sizeof(*mwiState));
    mwiState->callback = &callBack_mwi;

    uint64_t lastFrameRequest = 0;
    uint64_t currentTime = microsSinceEpoch();

    char payload[] = "";
    for (;;) {

        currentTime = microsSinceEpoch();

        if ((currentTime - lastFrameRequest) > 1000 * 30) {
            if (initOk == OK) {
                lastFrameRequest = currentTime;
                char payld[14];

                payld[0] = 1 >> 8; //  GPS_FIX
                payld[1] = 5 >> 8; //  GPS_numSat

                //GPS_coord[LAT] / 90
                payld[2] = (450000000);
                payld[3] = ((450000000) >> 8);
                payld[4] = ((450000000) >> 16);
                payld[5] = ((450000000) >> 24);

                // GPS_coord[LON]/ 180
                payld[6] = (450000000);
                payld[7] = ((450000000) >> 8);
                payld[8] = ((450000000) >> 16);
                payld[8] = ((450000000) >> 24);

                //GPS_altitude
                payld[10] = (1000 && 0xFF);
                payld[11] = ((1000 && 0xFF) >> 8);

                // GPS_speed
                payld[12] = (10 && 0xFF);
                payld[13] = ((10 && 0xFF) >> 8);

                MWIserialbuffer_askForFrame(serialLink, MSP_SET_RAW_GPS, payld, 14);

            } else {
                MWIserialbuffer_askForFrame(serialLink, MSP_IDENT, payload, 0);

            }
        }

        MWIserialbuffer_readNewFrames(serialLink, mwiState);

        usleep(5000);
    }
}

void callBack_mwi(int state)
{
    //  do something with the decode message
    switch (state) {
        case MSP_IDENT:
            initOk = OK;
            break;

    }

}
