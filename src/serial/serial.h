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
#include "../include/utils.h"

#define SERIAL_115200_BAUDRATE  115200
#define SERIAL_57600_BAUDRATE   57600
#define SERIAL_38400_BAUDRATE   38400
#define SERIAL_19200_BAUDRATE   19200
#define SERIAL_9600_BAUDRATE    9600

HANDLE serialport_init(const char* serialport, int i);

int serialport_writeChar(HANDLE fd, char b);
int serialport_write(HANDLE fd, char* str, int len);

int serialport_readChar(HANDLE fd, char* buf);
int serialport_readUntil(HANDLE fd, char* buf, char until);

