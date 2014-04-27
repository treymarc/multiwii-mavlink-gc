

VERBOSE := 2

ifdef ComSpec
WINBUILD := true
else
WINBUILD := false
endif

#--------------------

ifeq ($(WINBUILD),true)
DWIN := -D_WINDOZ
RM := erase
PATH_SEP := \\
EXE_SUFIX := .exe
else
RM := rm -r
PATH_SEP := /
endif

CC	?= gcc

CFLAGS	:= -std=c99 -pedantic -g -O3 

CFLAGS_MAVLINK = -DMAVLINK_EXTERNAL_RX_STATUS=0 -DMAVLINK_CHECK_MESSAGE_LENGTH=0

CFLAGS	+= $(DWIN) -D_LOGLEVEL=$(VERBOSE) $(CFLAGS_MAVLINK) -D_GNU_SOURCE  -Wall -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 


.PHONY: all clean 

all:

EXTRA	:= src$(PATH_SEP)example src$(PATH_SEP)mavlink
SRCDIRS	:= src$(PATH_SEP)serial src$(PATH_SEP)mwi $(EXTRA)

include $(SRCDIRS:%=%$(PATH_SEP)Module.mk)
