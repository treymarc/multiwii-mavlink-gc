#ifndef _MWGCCONF_H_
#define _MWGCCONF_H_

#include <stdio.h>
#include <stdint.h>
#include "../include/utils.h"
#include "mwgc.h"

#define TYPE_PX4 -1

void eexit(HANDLE code);
void rtfmHelp(void);
void rtfmVersion(const char * version);
int config(mavlink_state_t *mavlinkState,int argc, char* argv[]);

#endif
