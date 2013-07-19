##############################
# Rmotion Makefile
##############################
CC=gcc
APP_BINARY=Rmotion
CVLIBS = $(shell pkg-config --libs opencv)
CVFLAGS = $(shell pkg-config --cflags opencv)

CFLAGS += -I$(CVFLAGS)

OBJECTS= Rmotion.o
		

all:	Rmotion

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(APP_BINARY) core *~ log errlog *.avi

# Applications:
lockface:	$(OBJECTS)
	$(CC)	 $(CFLAGS) $(OBJECTS) \
		$(CVLIBS)\
		-lm\
		-o $(APP_BINARY)
			
	chmod 755 $(APP_BINARY)


