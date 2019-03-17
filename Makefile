CPPFLAGS = -std=c++11 -g -Wall -ggdb3
OPENCV = `pkg-config --cflags --libs opencv`

app: main.o ball_tracker.o pid.o serial.o
	g++ $(CPPFLAGS) -o "@" $^ $(OPENCV)

main.o: main.cpp ball_tracker/ball_tracker.h pid/pid.h serial/serial.h utils.h
	g++ $(CPPFLAGS) -c main.cpp $(OPENCV)

pid.o: pid/pid.c pid/pid.h utils.h
	gcc -c pid/pid.c

serial.o: serial/serial.c serial/serial.h utils.h
	gcc -c serial/serial.c

ball_tracker.o: ball_tracker/ball_tracker.cpp ball_tracker/ball_physic.h ball_physic.o utils.h
	g++ $(CPPFLAGS) -c ball_tracker/ball_tracker.cpp $(OPENCV)

ball_physic.o: ball_tracker/ball_physic.c ball_tracker/ball_physic.h utils.h
	gcc -c -lm ball_tracker/ball_physic.c


clean:
	-rm -f *.o
