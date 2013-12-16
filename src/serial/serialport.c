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
#include "serial.h"
#include <stdint.h>

#if defined( _WINDOZ )

HANDLE serialport_initWin(const char *portName);
serialport_writewin(HANDLE fd,char *buffer, unsigned int nbChar);

int serialport_readwin(HANDLE fd,char *buffer, unsigned int nbChar);

#else
#include <termios.h>
#endif

int serialport_writeChar(HANDLE fd, char b)

{

#if defined (_WINDOZ)
	int n = serialport_writewin(fd,b,1);
#else
	int n = write(fd, &b, 1);
#endif

	if (n != 1)

		return -1;

	return 0;

}

int serialport_write(HANDLE fd, const char* str)

{

	int len = 6;
	//int len = strlen(str);

#if defined (_WINDOZ)
	int n = serialport_writewin(fd,str,len);
#else
	int n = write(fd, str, len);
#endif

	if (n != len)

		return -1;

	return n;

}

int serialport_readChar(HANDLE fd, uint8_t* buf) {
	char b[1];
	int n = 0;

#if defined (_WINDOZ)
	n =serialport_readwin(fd,b, 1);
#else
	n = read(fd, b, 1);
#endif

	strcpy(buf, b);

	return (n == 1);

}

int serialport_readUntil(HANDLE fd, char* buf, char until) {
	char b[1];
	int i = 0;

	do {

#if defined (_WINDOZ)
		int n =serialport_readwin(fd,b, 1);
#else
		int n = read(fd, b, 1); // read a char at a time
#endif
		if (n == -1)
			return -1; // couldn't read

		if (n == 0) {
			usleep(10 * 1000); // wait 10 msec try again
			continue;
		}

		buf[i] = b[0];
		i++;

	} while (b[0] != until);

	buf[i] = 0; // null terminate the string
	return i;

}

HANDLE serialport_init(const char* serialport, int baudrate) {
	int fd;
#if defined( _WINDOZ ) // win
	fd = serialport_initWin(serialport);
#else // posix
	struct termios toptions;

	fd = open(serialport, O_RDWR | O_NOCTTY);

	if (fd == -1) {
		perror("init_serialport: Unable to open port ");
		return -1;
	}

	if (tcgetattr(fd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
		return -1;
	}

	// serial port default baud 115200 ,
	if (baudrate != SERIAL_DEFAULT_BAUDRATE) {
		cfsetispeed(&toptions, baudrate);
		cfsetospeed(&toptions, baudrate);
	} else {
		cfsetispeed(&toptions, B115200);
		cfsetospeed(&toptions, B115200);
	}

	// set parity8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN] = 0;
	toptions.c_cc[VTIME] = 0;

	// apply options
	if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return 0;
	}
#endif
	return fd;
}

//win

#if defined( _WINDOZ )

HANDLE serialport_initWin(const char *portName)
{

	//Try to open device
	HANDLE fd = CreateFile(portName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);

	//Check if the connection was successfull
	if(fd==INVALID_HANDLE_VALUE)
	{
		//If not success full display an Error

		perror("ERROR: Handle was not attached. \n");
		return -1;

	}

	//If connected we try to set the comm parameters
	DCB dcbSerialParams = {0};

	//Try to get the current
	if (!GetCommState(fd, &dcbSerialParams)) {
		//If impossible, show an error
		MW_TRACE("failed to get current serial parameters!");
		return -1;
	}

	//Define serial connection parameters for the arduino board
	dcbSerialParams.BaudRate=CBR_115200;
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;

	//Set the parameters and check for their proper application
	if(!SetCommState(fd, &dcbSerialParams)) {
		perror("ALERT: Could not set Serial Port parameters");
		return -1;

	}
	Sleep(1000);

	return fd;
}

int serialport_readwin(HANDLE fd,char *buffer, unsigned int nbChar)
{
	DWORD bytesRead;
	unsigned int toRead;
	COMSTAT status;
	DWORD errors;

	//Use the ClearCommError function to get status info on the Serial port
	ClearCommError(fd, &errors, &status);

	//Check if there is something to read
	if(status.cbInQue>0) {
		//If there is we check if there is enough data to read the required number
		//of characters, if not we'll read only the available characters to prevent
		//locking of the application.
		if(status.cbInQue>nbChar) {
			toRead = nbChar;
		} else {
			toRead = status.cbInQue;
		}

		//Try to read the require number of chars, and return the number of read bytes on success
		if(ReadFile(fd, buffer, toRead, &bytesRead, NULL) && bytesRead != 0) {
			return bytesRead;
		}
	}
	//If nothing has been read, or that an error was detected return -1
	return -1;
}

serialport_writewin(HANDLE fd,char *buffer, unsigned int nbChar)
{
	DWORD bytesSend;
	COMSTAT status;
	DWORD errors;
	//Try to write the buffer on the Serial port
	if(!WriteFile(fd, (void *)buffer, nbChar, &bytesSend, 0)) {
		//In case it don't work get comm error and return false
		ClearCommError(fd, &errors, &status);
		return -1;
	} else {
		return nbChar;
	}
}
#endif
