HEADERS = pid.h
CC = gcc

test: test.o pid.o
	$(CC) -o "$@" test.o pid.o

test.o: test.c $(HEADERS)
	$(CC) -c test.c pid.c

pid.o: pid.c $(HEADERS)
	$(CC) -c pid.c

clean:
	-rm -f pid.o
	-rm -f test.o
	-rm -f test
