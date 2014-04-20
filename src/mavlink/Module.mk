#
# MWGC 
# multiwii serial protocol to mavlink UDP  
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.


MAVLINK_SRC_DIR	:= src$(PATH_SEP)mavlink

MAVLINK_SRC_CFLAGS	:= -O2 

MAVLINK_SRC_TARGETS	:=  mwgc


#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif

$(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc: $(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc.o  $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o
	$(CC) $(MAVLINK_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#

$(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc.o: $(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc.c  
	$(CC)  $(MAVLINK_SRC_CFLAGS) $(CFLAGS) $(MAVLINK_SRC_CFLAGS) -c $< -o $@

#
#
# Commands
#

all-mwgc: $(addprefix $(MAVLINK_SRC_DIR)$(PATH_SEP),$(MAVLINK_SRC_TARGETS))


clean-mwgc:
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)*.o
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc$(EXE_SUFIX)
	

all: all-mwgc


clean: clean-mwgc


