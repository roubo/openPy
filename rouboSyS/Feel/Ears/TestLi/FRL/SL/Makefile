##############################
# Srecogn Makefile
# roubo_dai@san412.in
# San412
##############################
CC = gcc
APP_BINARY = Srecogn

CLIBS  = $(shell pkg-config --libs pocketsphinx sphinxbase)
CFLAGS = $(shell pkg-config --cflags pocketsphinx sphinxbase)
MODELS = $(shell pkg-config --variable=modeldir pocketsphinx)
VERSION = 0.2.1

OBJECTS = Srecogn.o
		
all:	Srecogn

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(APP_BINARY) core *~ log 

Srecogn:	$(OBJECTS)
	$(CC) 	$(CFLAGS) $(OBJECTS)\
		$(CLIBS)\
		-o $(APP_BINARY)
	
	chmod 755 $(APP_BINARY)


