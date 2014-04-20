#
# MWI 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.



MWI_SRC_DIR	:= src$(PATH_SEP)mwi
MWI_SRC_CFLAGS	:= -O2 
MWI_SRC_TARGETS	:=  mwi.o


#
# Programs
#
#
ifeq ($(WINBUILD),true)
	WINOP := -lws2_32 
endif


#
# Objects
#

$(MWI_SRC_DIR)$(PATH_SEP)mwi.o: $(MWI_SRC_DIR)$(PATH_SEP)mwi.c  
	$(CC) $(CFLAGS) $(MWI_SRC_CFLAGS) -c $< -o $@


#
#
# Commands
#

all-mwi: $(addprefix $(MWI_SRC_DIR)$(PATH_SEP),$(MWI_SRC_TARGETS))


clean-mwi:
	$(RM)  $(MWI_SRC_DIR)$(PATH_SEP)*.o
	

all: all-mwi


clean: clean-mwi


