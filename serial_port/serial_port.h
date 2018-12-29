
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdint.h>

#define     BAUD_RATE   9600
#define     ttyACM0     "/dev/ttyACM0"
#define     ttyACM1     "/dev/ttyACM1"
#define     ttyACM2		"/dev/ttyACM2"
#define     ttyACM3		"/dev/ttyACM3"
#define     ttyACM4		"/dev/ttyACM4"

#define     bool        int
#define     false       0
#define     true        1


typedef struct ServoConfig {
    uint8_t     ServoX;
    uint8_t     ServoY;
    uint8_t     flag1;
    uint8_t     flag2;
} ServoConfig_t;


//___funcion signature_________________________________
bool openSerialCommunication(int* fd);
void setSerialAttributes(int fd);
void encodeConfig(ServoConfig_t* config, uint8_t* buf);
void decodeConfig(uint8_t* buf, ServoConfig_t* config);
void printServoConfig(ServoConfig_t* config);
void printEncodedPack(uint8_t* buf);


#endif