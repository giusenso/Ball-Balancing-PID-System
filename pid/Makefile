CC = gcc

test: test.c pid.c
	$(CC) -Wall -g -o "$@" test.c pid.c

test.o: test.c pid.h
	$(CC) -c test.c pid.c

pid.o: pid.c pid.h ../utils.h
	$(CC) -lm -c pid.c


clean:
	-rm -f pid.o
	-rm -f pid_test.o
	-rm -f test




