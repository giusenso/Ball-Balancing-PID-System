g++ -std=c++11 -g -Wall -ggdb3 serial_port/serial_port.c pid/pid.c ball_tracker/ball_tracker.c test.cpp -o test `pkg-config --cflags --libs opencv`
