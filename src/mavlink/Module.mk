#
# MWGC 
# multiwii serial protocol to mavlink UDP  
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

MAVLINK_SRC_DIR	:= src$(PATH_SEP)mavlink
MAVLINK_SRC_CFLAGS	:=
MAVLINK_SRC_OBJECT :=  	mwgc
MAVLINK_SRC_TARGETS	:=  $(MAVLINK_SRC_OBJECT)-$(MWGC_VERSION)

#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif

$(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_TARGETS): $(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_OBJECT).o  $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o $(MAVLINK_SRC_DIR)$(PATH_SEP)man.o
	$(CC) $(MAVLINK_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#
$(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_OBJECT).o: $(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc.c  
	$(CC)  $(MAVLINK_SRC_CFLAGS) $(CFLAGS) $(MAVLINK_SRC_CFLAGS) -c $< -o $@

$(MAVLINK_SRC_DIR)$(PATH_SEP)man.o: $(MAVLINK_SRC_DIR)$(PATH_SEP)man.c  
	$(CC)  $(MAVLINK_SRC_CFLAGS) $(CFLAGS) $(MAVLINK_SRC_CFLAGS) -c $< -o $@
#
#
# Commands
#
all-mwgc: $(addprefix $(MAVLINK_SRC_DIR)$(PATH_SEP),$(MAVLINK_SRC_TARGETS))

clean-mwgc:
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)*.o
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_TARGETS)*$(EXE_SUFIX)
	

all: all-mwgc

clean: clean-mwgc

