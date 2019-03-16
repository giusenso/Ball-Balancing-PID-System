CPPFLAGS = -std=c++11 -g -Wall -ggdb3
OPENCV = `pkg-config --cflags --libs opencv`

app: main.o ball_tracker.o pid.o serial.o
	g++ -o app main.o ball_tracker.o pid.o serial.o

main.o: main.cpp ball_tracker/ball_tracker.h pid/pid.h serial/serial.h
	g++ $(CPPFLAGS) -c main.cpp $(OPENCV)

ball_tracker.o: ball_tracker/ball_tracker.cpp ball_tracker/ball_physic.h
	g++ $(CPPFLAGS) -c ball_tracker/ball_tracker.cpp $(OPENCV)

ball_physic.o: ball_tracker/ball_physic.c ball_tracker/ball_physic.h
	gcc -c -lm ball_tracker/ball_physic.c

pid.o: pid/pid.c pid/pid.h
	gcc -c pid/pid.c

serial.o: serial/serial.c serial/serial.h
	gcc -c serial/serial.c


clean:
	-rm -f *.o
