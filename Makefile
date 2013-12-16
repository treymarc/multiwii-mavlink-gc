

LOGLEV := 3
WINBUILD := false

#WINBUILD := true


#--------------------

ifeq ($(WINBUILD),true)
DWIN := -D_WINDOZ
endif

CC	?= gcc  

CFLAGS	?= -g

#  debug
CFLAGS	:= -O2 -g -std=c99

CFLAGS	+= $(DWIN) -D_LOGLEVL=$(LOGLEV) -D_GNU_SOURCE -Wall -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 

RM		:= rm -f

.PHONY: all  clean 

all:

EXTRA	:= src/example
SRCDIRS	:=    src/serial src/mwgc $(EXTRA)

include $(SRCDIRS:%=%/Module.mk)
