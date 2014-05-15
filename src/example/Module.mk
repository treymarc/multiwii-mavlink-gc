#
# MUltiwii serial protocol example  
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

EXAMPLE_SRC_DIR	:= src$(PATH_SEP)example
EXAMPLE_SRC_CFLAGS	:=  
EXAMPLE_SRC_TARGETS	:= example log2csv gps


$(EXAMPLE_SRC_DIR)$(PATH_SEP)gps: $(EXAMPLE_SRC_DIR)$(PATH_SEP)gps.o $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o $(UTILS_SRC_DIR)$(PATH_SEP)utils.o
	$(CC) $(EXAMPLE_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


$(EXAMPLE_SRC_DIR)$(PATH_SEP)example: $(EXAMPLE_SRC_DIR)$(PATH_SEP)example.o $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o $(UTILS_SRC_DIR)$(PATH_SEP)utils.o
	$(CC) $(EXAMPLE_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)

$(EXAMPLE_SRC_DIR)$(PATH_SEP)log2csv: $(EXAMPLE_SRC_DIR)$(PATH_SEP)log2csv.o $(SERIAL_SRC_DIR)$(PATH_SEP)serialport.o $(MWI_SRC_DIR)$(PATH_SEP)mwi.o $(UTILS_SRC_DIR)$(PATH_SEP)utils.o
	$(CC) $(EXAMPLE_SRC_CFLAGS) $(LDFLAGS) -o $@ $^ $(WINOP)


#
# Objects
#
$(EXAMPLE_SRC_DIR)$(PATH_SEP)example.o: $(EXAMPLE_SRC_DIR)$(PATH_SEP)example.c  
	$(CC) $(CFLAGS) $(EXAMPLE_SRC_CFLAGS) -c $< -o $@

$(EXAMPLE_SRC_DIR)$(PATH_SEP)log2csv.o: $(EXAMPLE_SRC_DIR)$(PATH_SEP)log2csv.c  
	$(CC) $(CFLAGS) $(EXAMPLE_SRC_CFLAGS) -c $< -o $@

$(EXAMPLE_SRC_DIR)$(PATH_SEP)gps.o: $(EXAMPLE_SRC_DIR)$(PATH_SEP)gps.c  
	$(CC) $(CFLAGS) $(EXAMPLE_SRC_CFLAGS) -c $< -o $@

#
#
# Commands
#
all-example: $(addprefix $(EXAMPLE_SRC_DIR)$(PATH_SEP),$(EXAMPLE_SRC_TARGETS))

clean-example:
	-$(RM)  $(EXAMPLE_SRC_DIR)$(PATH_SEP)*.o
	-$(RM)  $(EXAMPLE_SRC_DIR)$(PATH_SEP)example$(EXE_SUFIX)
	-$(RM)  $(EXAMPLE_SRC_DIR)$(PATH_SEP)log2csv$(EXE_SUFIX)
	-$(RM)  $(EXAMPLE_SRC_DIR)$(PATH_SEP)gps$(EXE_SUFIX)

all: all-example

clean: clean-example

