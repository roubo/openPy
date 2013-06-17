##############################
# spcaview Makefile
##############################

INSTALLROOT=$(PWD)

CC=gcc
CPP=g++
INSTALL=install
APP_BINARY=litearry
BIN=/usr/local/bin
SDLLIBS = $(shell sdl-config --libs) 
SDLFLAGS = $(shell sdl-config --cflags)
CVLIBS = $(shell pkg-config --libs opencv)
CVFLAGS = $(shell pkg-config --cflags opencv)
#LIBX11FLAGS= -I/usr/X11R6/include -L/usr/X11R6/lib
VERSION = 0.2.1

#WARNINGS = -Wall \
#           -Wundef -Wpointer-arith -Wbad-function-cast \
#           -Wcast-align -Wwrite-strings -Wstrict-prototypes \
#           -Wmissing-prototypes -Wmissing-declarations \
#           -Wnested-externs -Winline -Wcast-qual -W \
#           -Wno-unused
#           -Wunused

CFLAGS += -I$(SDLFLAGS) $(WARNINGS) $(CVFLAGS)
CPPFLAGS = $(CFLAGS)

OBJECTS= litearry.o
		

all:	litearry

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(APP_BINARY) core *~ log errlog *.avi

# Applications:
litearry:	$(OBJECTS)
	$(CC)	$(CFLAGS) $(OBJECTS) $(X11_LIB) $(XPM_LIB)\
		$(MATH_LIB) \
		$(SDLLIBS)\
		$(CVLIBS)\
		-lm\
		-o $(APP_BINARY)
			
	chmod 755 $(APP_BINARY)


install: luvcview
	$(INSTALL) -s -m 755 -g root -o root $(APP_BINARY) $(BIN) 
	rm -f $(BIN)/uvcview
