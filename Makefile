
#--------------------
# User Options
#--------------------

VERSION = 1.0
VERBOSE = 3
MAVLINK = 1.0

#--------------------
# Platform options
#--------------------

ifdef ComSpec
WINBUILD = true
DWIN = -D_WINDOZ
RM = erase
PATH_SEP = \\
EXE_SUFIX = .exe
#BUILDDATE = $(shell (DATE /T yymmdd))
else
WINBUILD= false
RM = rm -r
PATH_SEP = /
#BUILDDATE = $(shell (date +'%Y%m%d'))
endif


#MWGC_VERSION = $(VERSION)-$(BUILDDATE)
MWGC_VERSION = $(VERSION)


#--------------------
#  Compiler options
#--------------------


CC	?= gcc

CFLAGS	= -std=c99 -pedantic -g -O1

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
