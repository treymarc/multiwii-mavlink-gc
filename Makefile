
#--------------------
# User Options
#--------------------

VERSION = 1.1
VERBOSE = 2
MAVLINK = 1.0

#--------------------
# Platform options
#--------------------
ifeq ($(shell uname), Linux)
WINBUILD= false
#BUILDDATE = $(shell (date +'%Y%m%d'))
else
WINBUILD = true
DWIN = -D_WINDOZ
EXE_SUFIX = .exe
#BUILDDATE = $(shell (DATE /T yymmdd))
endif

RM = rm -r
PATH_SEP = /

#MWGC_VERSION = $(VERSION)-$(BUILDDATE)
MWGC_VERSION = $(VERSION)


#--------------------
#  Compiler options
#--------------------


CFLAGS	= -std=gnu99 -pedantic -g -O1

CFLAGS_MAVLINK = -I./lib/mavlink/$(MAVLINK)/  -DMAVLINK_EXTERNAL_RX_STATUS=0 -DMAVLINK_EXTERNAL_RX_BUFFER=0 -DMAVLINK_CHECK_MESSAGE_LENGTH=0

CFLAGS_MWGC = -D_LOGLEVEL=$(VERBOSE) -D_MWGC_VERSION=\"$(MWGC_VERSION)\"

CFLAGS	+= $(DWIN) $(CFLAGS_MAVLINK) $(CFLAGS_MWGC) -D_GNU_SOURCE  -Wall -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 


#--------------------
# Build modules
#--------------------

.PHONY: all clean 

all:

EXTRA	:= src$(PATH_SEP)example src$(PATH_SEP)mwgc
SRCDIRS	:= src$(PATH_SEP)utils src$(PATH_SEP)serial src$(PATH_SEP)mwi $(EXTRA)

include $(SRCDIRS:%=%$(PATH_SEP)Module.mk)
