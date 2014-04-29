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
#ifndef _MWGCH_H_
#define _MWGCH_H_

typedef struct {

    char targetIp[150];
    char serialDevice[150];
    int baudrate;
    uint32_t hertz;

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

#endif
