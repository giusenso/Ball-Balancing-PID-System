HEADERS = serial_port.h
CC = gcc

test: test.o serial_port.o
	$(CC) -o "$@" test.o serial_port.o

test.o: test.c $(HEADERS)
	$(CC) -c test.c serial_port.c

serial_port.o: serial_port.c $(HEADERS)
	$(CC) -c serial_port.c

clean:
	-rm -f serial_port.o
	-rm -f test.o
	-rm -f test