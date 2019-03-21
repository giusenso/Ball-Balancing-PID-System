HEADERS = serial.h
CC = gcc

test: test.o serial.o
	$(CC) -Wall -o "$@" test.o serial.o -lm

test.o: test.c $(HEADERS)
	$(CC) -c test.c serial.c

serial.o: serial.c $(HEADERS) ../utils.h
	$(CC) -c serial.c 

clean:
	-rm -f serial.o
	-rm -f test.o
	-rm -f test
