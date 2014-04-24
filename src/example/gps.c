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

    msp_payload_t *payload;
    payload = calloc(sizeof(*payload), sizeof(*payload));

    for (;;) {

        currentTime = microsSinceEpoch();

        if ((currentTime - lastFrameRequest) > 1000 * 30) {
            if (initOk == OK) {
                lastFrameRequest = currentTime;

                payload->length = 0;
                MWIserialbuffer_Payloadwrite8(payload, 1);          // GPS_FIX
                MWIserialbuffer_Payloadwrite8(payload, 5);          // GPS_numSat
                MWIserialbuffer_Payloadwrite32(payload, 4500000);   // GPS_coord[LAT] / 90
                MWIserialbuffer_Payloadwrite32(payload, 4500000);   // GPS_coord[LON]/ 180
                MWIserialbuffer_Payloadwrite16(payload, 1000);      // GPS_altitude
                MWIserialbuffer_Payloadwrite16(payload, 1000);      // GPS_speed

                MWIserialbuffer_askForFrame(serialLink, MSP_SET_RAW_GPS, payload);

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
    //  do something with the decode message
    switch (state) {
        case MSP_IDENT:
            initOk = OK;
            break;

    }

}
