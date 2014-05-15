#
# MWGC 
# multiwii serial protocol to mavlink UDP  
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

MAVLINK_SRC_DIR	:= src$(PATH_SEP)mwgc
MAVLINK_SRC_OBJECT :=  	mwgc
MAVLINK_SRC_TARGETS	:=  $(MAVLINK_SRC_OBJECT)-$(MWGC_VERSION)$(EXE_SUFIX)

#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif

$(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_TARGETS): $(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_OBJECT).o  $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o $(UTILS_SRC_DIR)$(PATH_SEP)utils.o $(MAVLINK_SRC_DIR)$(PATH_SEP)conf.o
	$(CC) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#
$(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_OBJECT).o: $(MAVLINK_SRC_DIR)$(PATH_SEP)mwgc.c  
	$(CC)  $(MAVLINK_SRC_CFLAGS) $(CFLAGS) $(MAVLINK_SRC_CFLAGS) -c $< -o $@

$(MAVLINK_SRC_DIR)$(PATH_SEP)conf.o: $(MAVLINK_SRC_DIR)$(PATH_SEP)conf.c  
	$(CC)  $(MAVLINK_SRC_CFLAGS) $(CFLAGS) $(MAVLINK_SRC_CFLAGS) -c $< -o $@
#
#
# Commands
#
all-mwgc: $(addprefix $(MAVLINK_SRC_DIR)$(PATH_SEP),$(MAVLINK_SRC_TARGETS))

clean-mwgc:
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)*.o
	$(RM)  $(MAVLINK_SRC_DIR)$(PATH_SEP)$(MAVLINK_SRC_TARGETS)
	

all: all-mwgc

clean: clean-mwgc

