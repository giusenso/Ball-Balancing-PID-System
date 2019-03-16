CC = gcc

all: test

test: test.o pid.o
	$(CC) -Wall -g pid.o test.o -o test

test.o: test.c
	$(CC) -c test.c

pid.o: pid.c
	$(CC) -lm -c pid.c


clean:
	-rm -f pid.o
	-rm -f pid_test.o
	-rm -f test




