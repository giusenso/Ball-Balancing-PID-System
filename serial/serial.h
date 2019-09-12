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

/* baud rates */
#define	ATMEGA2560_BAUD_RATE	B9600
#define ESP32_BAUD_RATE			B115200

/* serial ports */
#define     ttyACM0     "/dev/ttyACM0"
#define     ttyACM1     "/dev/ttyACM1"
#define     ttyACM2		"/dev/ttyACM2"
#define     ttyACM3		"/dev/ttyACM3"
#define     ttyACM4		"/dev/ttyACM4"

#define     ttyUSB0     "/dev/ttyUSB0"
#define     ttyUSB1     "/dev/ttyUSB1"
#define     ttyUSB2    	"/dev/ttyUSB2"
#define     ttyUSB3     "/dev/ttyUSB3"
#define     ttyUSB4     "/dev/ttyUSB4"


const extern char* serial_ports[5];
extern int baud_rate;

//___funcion signature_________________________________
bool set_mcu(int _mcu);
int openSerialCommunication(int* fd);
void setSerialAttributes(int fd);
void encodeConfig(Point_t* ball_pos, uint8_t* buf);
void closeSerialCommunication(int* fd, Point_t* ball);

#endif
