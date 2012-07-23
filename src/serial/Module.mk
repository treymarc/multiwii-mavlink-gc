# 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.


SERIAL_SRC_DIR	:= src/serial

$(SERIAL_SRC_DIR)/serialport.o: $(SERIAL_SRC_DIR)/serialport.c  
	$(CC) $(CFLAGS) $(SERIAL_SRC_CFLAGS) -c $<  -o $@




#
#
# Commands
#

all-serial: $(SERIAL_SRC_DIR)/serialport.o 


clean-serial:
	$(RM)  $(SERIAL_SRC_DIR)/*.o 


all: all-serial


clean: clean-serial
