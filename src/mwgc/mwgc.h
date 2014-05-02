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

typedef struct {

    char targetIp[150];
    char serialDevice[150];
    int baudrate;
    int hertz;

    int autoTelemtry, calibrating, sendRcData;
    struct rcdata {
        int x, y, z, r, buttons;
        int toSend;
    } rcdata;

    int mwiUavID;
    int mwiAutoPilotType;
    int mwiFlightMode;
    int mwiAirFrametype;
} mavlink_state_t;


#include <stdio.h>
#include <stdint.h>
#include "../utils/utils.h"
// mavlink message headers
#include "../mavlink/common/mavlink.h"


#define TYPE_PX4 -1

void eexit(HANDLE code);
void rtfmHelp(void);
void rtfmVersion(const char * version);
int config(mavlink_state_t *mavlinkState,int argc, char* argv[]);

