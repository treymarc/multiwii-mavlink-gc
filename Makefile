

LOGLEV :=3
WINBUILD := false

#WINBUILD := true


#--------------------

ifeq ($(WINBUILD),true)
DWIN := -D_WINDOZ
endif

CC	?= gcc

CFLAGS	:= -std=c99 -g 

CFLAGS	+= $(DWIN) -D_LOGLEVL=$(LOGLEV) -D_GNU_SOURCE -Wall -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 

RM		:= rm -f

.PHONY: all clean 

all:

EXTRA	:= src/example src/mavlink
SRCDIRS	:= src/serial src/mwi $(EXTRA)

include $(SRCDIRS:%=%/Module.mk)
