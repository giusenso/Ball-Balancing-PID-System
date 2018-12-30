#Makefile for testing
                                   
HEADERS = ball_tracker/ball_tracker.h serial_port/serial_port.h pid/pid.h


test: test.o ball_tracker.o serial_port.o pid.o
	g++ -o "$@" test.o ball_tracker.o serial_port.o pid.o

test.o: test.c $(HEADERS)
	g++ -c ball_tracker.c serial_port.c pid.c test.c
        
ball_tracker/ball_tracker.o: ball_tracker/ball_tracker.c ball_tracker/ball_tracker.h
	g++ -c ball_tracker/ball_tracker.c `pkg-config --cflags --libs opencv`

serial_port/serial_port.o: serial_port.c serial_port.h
	gcc -c serial_port/serial_port.c

pid/pid.o: pid.c pid.h
	gcc -c pid/pid.c


clean:
	-rm -f ball_tracker/ball_tracker.o        
	-rm -f serial_port/serial_port.o
	-rm -f pid/pid.o
	-rm -f test.o
	-rm -f test
