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
 ****************************************************************************/
typedef enum {
    FC_BASEFLIGHT =0,
    FC_MWI8BIT ,
} fcType;

typedef struct {

    // not related to the mav state
    char targetIp[150];         // target ip , should be 127.0.0.1 , gs and fs must be the same
    char serialDevice[150];     // serial device name
    int baudrate;
    int hertz;                  // msp update
    int verbose;                // verbose level , 0 = no output
    int hil;                    // hardware in the loop simulation , need Flight simulator


    // related to the mav
    int autoTelemtry, calibrating;

    int throttleHalfRange;
    int sendRcData;             // true if the gs send rcdata
    struct rcdata {             // mavlink rcdata
        int x, y, z, r, buttons;
        int toSend;
    } rcdata;

    int fcType;
    int mwiUavID;
    int mwiAutoPilotType;       // this is used to report as a px4 or generic in the config screen
    int mwiFlightMode;          // the reported mavlink flight mode
    int mwiAirFrametype;        // the reported mavlink airframe mode
} mavlink_state_t;


#include <stdio.h>
#include <stdint.h>
#include "../utils/utils.h"



#define TYPE_PX4 -1

void eexit(HANDLE code);
void rtfmHelp(void);
void rtfmVersion(const char * version);
int config(mavlink_state_t *mavlinkState,int argc, char* argv[]);

