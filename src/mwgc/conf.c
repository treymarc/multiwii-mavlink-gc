#include "mwgc.h"
#include <string.h>
#include <stdlib.h>

void rtfmVersion(const char * version)
{
    printf("\nVersion: %s", version);
    printf("\n\n");
}

void rtfmHelp(void)
{
    printf("\nMultiwii Mavlink groundControl\n");

    printf("\n Usage:\n\n");

    printf("\t -sendrcdata : send rc command to the fligt controller\n");
    printf("\t  1 : send msp command\n");
    printf("\t  0 : dont send command\n\n");
    printf("\t -throttlerange : the joystick throttle value range\n");
    printf("\t  1 : expected only positive value [0,1]\n");
    printf("\t  0 : default is full range[-1,1]\n\n");
    printf("\t -hil : hardware in the loop simulation\n");
    printf("\t  1 : flight gear simulator\n");
    printf("\t  0 : no simulation\n\n");
    printf("\t -v : verbose level default is 1\n");
    printf("\t  1 : normal\n");
    printf("\t  0 : quiet\n\n");
    printf("\t -ip : ip address of QGroundControl\n");
    printf("\t  default value : 127.0.0.1\n\n");
    printf("\t -s : serial device name\n");
    printf("\t  default value : /dev/ttyO2\n\n");
    printf("\t -baudrate : baudrate to use woth serial link\n");
    printf("\t  default value : 115200\n\n");
    printf("\t -id : id for this flight controller in mavlink\n");
    printf("\t  default value : 1\n\n");
    printf("\t -px4 : px4 mode , only for the configuration screen in qgc\n");
    printf("\t  1 : claim to be a PX4 autopilote\n");
    printf("\t  0 : claim to be generic autopilot (default)\n\n");
    printf("\t -telemetryauto <n>\n");
    printf("\t  1 : assume the flight controler will send data\n");
    printf("\t  0 : send request for each data (default)\n\n");
    printf("\t -hertz <n>\n");
    printf("\t  Serial refresh for the imu. Default is 30, max is 60\n\n");
    printf("\t --help\n");
    printf("\t  display this message\n\n");
    printf("\t --version\n");
    printf("\t  show version number\n\n");
}

int config(mavlink_state_t *mavlinkState, int argc, char* argv[])
{
    if ((argc == 2) && (strcmp(argv[1], "--help") == 0)) {
        rtfmHelp();
        return NOK;
    } else if ((argc == 2) && (strcmp(argv[1], "--version") == 0)) {
        rtfmVersion(MWGC_VERSION);
        return NOK;
    } else {
        mavlinkState->verbose = 1;
        // parse options flags
        for (int i = 1; i < argc; i++) {
            if (i + 1 != argc) {
                if (strcmp(argv[i], "-ip") == 0) {
                    strcpy(mavlinkState->targetIp, argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-s") == 0) {
                    strcpy(mavlinkState->serialDevice, argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-id") == 0) {
                    mavlinkState->mwiUavID = atoi(argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-autotelemetry") == 0) {
                    mavlinkState->autoTelemtry = atoi(argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-baudrate") == 0) {
                    mavlinkState->baudrate = atoi(argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-hertz") == 0) {
                    mavlinkState->hertz = atoi(argv[i + 1]);
                    if ((mavlinkState->hertz < 1) || mavlinkState->hertz > 60)
                        mavlinkState->hertz = 60;
                    i++;
                } else if (strcmp(argv[i], "-sendrcdata") == 0) {
                    mavlinkState->sendRcData = atoi(argv[i + 1]);
                    i++;
                } else if (strcmp(argv[i], "-px4") == 0) {
                    if (atoi(argv[i + 1]) != 0) {
                        mavlinkState->mwiAutoPilotType = TYPE_PX4;
                    }
                    i++;
                } else if (strcmp(argv[i], "-v") == 0) {
                    mavlinkState->verbose = atoi(argv[i + 1]);
                    i++;
                }else if (strcmp(argv[i], "-hil") == 0) {
                    mavlinkState->hil = atoi(argv[i + 1]);
                    i++;
                }else if (strcmp(argv[i], "-throttlerange") == 0) {
                    mavlinkState->throttleHalfRange = atoi(argv[i + 1]);
                    i++;
                }

            }
        }
    }
    if (mavlinkState->verbose) {
        printf("ground station ip: %s\n", mavlinkState->targetIp);
        printf("serial link: %s\n", mavlinkState->serialDevice);
        printf("uav id: %i\n", mavlinkState->mwiUavID);
    }
    return OK;
}
