#include "man.h"

void rtfmVersion(const char * version)
{
    printf("\n\nVersion: ");
    printf(version);
    printf("\n\n");
}

void rtfmHelp(void)
{
    printf("\n\nmwgc - serial 2 udp");
    printf("\n");
    printf("\nUsage:\n\n");

    printf("\t -ip <ip address of QGroundControl>\n");
    printf("\t  default value : 127.0.0.1\n\n");
    printf("\t -s <serial device name>\n");
    printf("\t  default value : /dev/ttyO2\n\n");
    printf("\t -baudrate <spedd\n");
    printf("\t  default value : 115200\n\n");
    printf("\t -id <id for mavlink>\n");
    printf("\t  default value : 1\n\n");
    printf("\t  -telemetryauto <int>\n");
    printf("\t   1 : assume the flight controler will send data\n");
    printf("\t   0 : send request for each data (default)\n\n");
    printf("\t  -hertz <int>\n");
    printf("\t   Serial refresh for the imu. Default is 30, max is 60\n\n");
    printf("\t--help");
    printf("\t  display this message\n\n");
    printf("\t --version");
    printf("\t  show version number\n\n");
}

