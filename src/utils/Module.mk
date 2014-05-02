# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

UTILS_SRC_DIR	:= src$(PATH_SEP)utils

$(UTILS_SRC_DIR)$(PATH_SEP)utils.o: $(UTILS_SRC_DIR)$(PATH_SEP)utils.c  
	$(CC) $(CFLAGS) $(UTILS_SRC_CFLAGS) -c $<  -o $@

#
#
# Commands
#
all-utils: $(UTILS_SRC_DIR)$(PATH_SEP)utils.o 

clean-utils:
	$(RM)  $(UTILS_SRC_DIR)$(PATH_SEP)*.o 

all: all-utils

clean: clean-utils
