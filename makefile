# makefile for BME280. september 2018 / paulvha

CC = gcc
DEPS = BME280.h bcm2835.h twowire.h
OBJ = bme280_lib.o bme280.o 
LIBS = -lm -ltwowire -lbcm2835 

.cpp.o: %c $(DEPS)
	$(CC) -Wall -Werror -c -o $@ $<

bme280 : $(OBJ)
	$(CC) -o $@ $^ $(LIBS)

.PHONY : clean

clean :
	rm bme280 $(OBJ)
