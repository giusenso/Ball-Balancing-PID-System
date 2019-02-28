#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include "../utils.h"

#include <stdint.h>

#define BAUD_RATE 9600
#define MYUBRR (F_CPU/16/BAUD_RATE-1)

#define     ttyACM0     "/dev/ttyACM0"
#define     ttyACM1     "/dev/ttyACM1"
#define     ttyACM2		"/dev/ttyACM2"
#define     ttyACM3		"/dev/ttyACM3"
#define     ttyACM4		"/dev/ttyACM4"

/*********************************************/
/*      SERVOS                               */
/*********************************************/
#define   PERIOD          1/F_CPU
#define   DEAD_BAND       3   *0.000001
#define   HALF_DEGREE     3.5 *0.000001

#define   MIN_STEP          HALF_DEGREE/PERIOD   //56
#define   HALF_ANGLE        23200
#define   ANGLE_OFFSET      12000
#define	  MAX_ANGLE         HALF_ANGLE+ANGLE_OFFSET
#define   MIN_ANGLE         HALF_ANGLE-ANGLE_OFFSET
/*********************************************/


#define     bool        int
#define     false       0
#define     true        1


extern const char* serialPorts[5];

//___funcion signature_________________________________
int openSerialCommunication(int* fd);
void setSerialAttributes(int fd);
void encodeConfig(ServoConfig_t* config, uint8_t* buf);
void decodeConfig(uint8_t* buf, ServoConfig_t* config);
bool handShake(uint8_t* buf);
void printServoConfig(ServoConfig_t* config);
void printEncodedPack(uint8_t* buf);


#endif
