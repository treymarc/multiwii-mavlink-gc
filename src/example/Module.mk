#
# example UART 2 UDP 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.


EXAMPLE_SRC_DIR	:= src/example

EXAMPLE_SRC_CFLAGS	:=  

EXAMPLE_SRC_TARGETS	:=  example


#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif

$(EXAMPLE_SRC_DIR)/example: $(EXAMPLE_SRC_DIR)/example.o   $(SERIAL_SRC_DIR)/serialport.o $(UART2UDP_SRC_DIR)/mwi.o
	$(CC) $(EXAMPLE_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#

$(EXAMPLE_SRC_DIR)/example.o: $(EXAMPLE_SRC_DIR)/example.c  
	$(CC) $(CFLAGS) $(EXAMPLE_SRC_CFLAGS) -c $< -o $@


#
#
# Commands
#

all-example: $(addprefix $(EXAMPLE_SRC_DIR)/,$(EXAMPLE_SRC_TARGETS))


clean-example:
	$(RM)  $(EXAMPLE_SRC_DIR)/*.o
	$(RM)  $(EXAMPLE_SRC_DIR)/example  
	

all: all-example


clean: clean-example


