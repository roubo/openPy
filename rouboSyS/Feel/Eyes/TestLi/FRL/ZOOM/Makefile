##############################
# zoom Makefile
##############################

CC=gcc
APP_BINARY=zoom

SDLLIBS = $(shell sdl-config --libs)
SDLFLAGS = $(shell sdl-config --cflags)
CVLIBS = $(shell pkg-config --libs opencv)
CVFLAGS = $(shell pkg-config --cflags opencv)


CFLAGS += -I $(SDLFLAGS) $(CVFLAGS)
OBJECTS=zoom.o

all:	zoom

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(APP_BINARY) core *~ log errlog *.avi

# Applications:
zoom:	$(OBJECTS)
	$(CC)	 $(CFLAGS) $(OBJECTS) \
		$(CVLIBS)\
		$(SDLLIBS)\
		-lm\
		-o $(APP_BINARY)
			
	chmod 755 $(APP_BINARY)


