CC := gcc
CFLAGS := -Wall -W -O2

all: lemon_squashd

clean:
	-rm *.o lemon_squashd

lemon_squashd.o: lemon_squashd.c sensor_cmd.h
lemon_squashd: lemon_squashd.o
