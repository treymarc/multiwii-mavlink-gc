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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "../include/utils.h"

#define SERIAL_DEFAULT_BAUDRATE 0

HANDLE serialport_init(const char* serialport, int i);

int serialport_writeChar(HANDLE fd, char b);
int serialport_write(HANDLE fd, const char* str);

int serialport_readChar(HANDLE fd, uint8_t* buf);
int serialport_readUntil(HANDLE fd, char* buf, char until);

