#
# MWGC UART 2 UDP 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.


UART2UDP_SRC_DIR	:= src/mwgc

UART2UDP_SRC_CFLAGS	:=  

UART2UDP_SRC_TARGETS	:=  mwgc


#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif

$(UART2UDP_SRC_DIR)/mwgc: $(UART2UDP_SRC_DIR)/uart2udp.o  $(SERIAL_SRC_DIR)/serialport.o $(UART2UDP_SRC_DIR)/mwi.o
	$(CC) $(UART2UDP_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#

$(UART2UDP_SRC_DIR)/uart2udp.o: $(UART2UDP_SRC_DIR)/uart2udp.c  
	$(CC) $(CFLAGS) $(UART2UDP_SRC_CFLAGS) -c $< -o $@

$(UART2UDP_SRC_DIR)/mwi.o: $(UART2UDP_SRC_DIR)/mwi.c  
	$(CC) $(CFLAGS) $(UART2UDP_SRC_CFLAGS) -c $< -o $@


#
#
# Commands
#

all-mwgc: $(addprefix $(UART2UDP_SRC_DIR)/,$(UART2UDP_SRC_TARGETS))


clean-mwgc:
	$(RM)  $(UART2UDP_SRC_DIR)/*.o
	$(RM)  $(UART2UDP_SRC_DIR)/mwgc  
	

all: all-mwgc


clean: clean-mwgc


