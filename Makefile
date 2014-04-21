

LOGLEV :=3

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

CFLAGS	:= -std=c99 -g -O3 

CFLAGS	+= $(DWIN) -D_LOGLEVL=$(LOGLEV) -D_GNU_SOURCE -Wall -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 


.PHONY: all clean 

all:

EXTRA	:= src$(PATH_SEP)example src$(PATH_SEP)mavlink
SRCDIRS	:= src$(PATH_SEP)serial src$(PATH_SEP)mwi $(EXTRA)

include $(SRCDIRS:%=%$(PATH_SEP)Module.mk)
