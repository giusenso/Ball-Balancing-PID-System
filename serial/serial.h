/**
* @file serial.h
* @author Giuseppe Sensolini

* @brief SERIAL COMMUNICATION
* 		- open serial communication
*		- close serial communication
*		- packet encode/decode
*		- debug print
*
* @date 2019-01-11
* 
* @copyright Copyright (c) 2019
* 
*/

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include "../utils.h"

#define BAUD_RATE 9600
#define MYUBRR (F_CPU/16/BAUD_RATE-1)

#define     ttyACM0     "/dev/ttyACM0"
#define     ttyACM1     "/dev/ttyACM1"
#define     ttyACM2		"/dev/ttyACM2"
#define     ttyACM3		"/dev/ttyACM3"
#define     ttyACM4		"/dev/ttyACM4"

extern const char* serialPorts[5];


//___funcion signature_________________________________
int openSerialCommunication(int* fd);
void closeSerialCommunication(int* fd, Point_t* ball);
void setSerialAttributes(int fd);

#endif
