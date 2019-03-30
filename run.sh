
#optimization level: fast
g++ -std=c++11 -g  -Wall -pg -ggdb3 serial/serial.c pid/pid.c ball_tracker/ball_physic.c ball_tracker/ball_tracker.cpp settings/file_handler.c main.cpp -Ofast -o test `pkg-config --cflags --libs opencv` -lpthread

