/**
* @file serial.c
* @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
*
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


/********************************************
*	@Author: Giuseppe Sensolini Arra'		*
*	@Date: 01.2019							*
*											*
*	SERIAL COMMUNICATION MODULE				*
*	task:									*
*		- open serial communication			*
*		- close serial communication		*
*		- handshake routine					*
*		- encode/decode packs				*
*		- write/read data					*
*		- debug print 						*
*											*
*********************************************/

#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>

#include "serial.h"

/*=============================================================*/

int baud_rate;
const char* serial_ports[5];

/**
 * @brief Set baudrate and serials ports based on the microcontreller model
 * 
 * @param _mcu 
 * @return true 
 * @return false 
 */
bool set_mcu(int _mcu){
	if( _mcu == ATMEGA2560 ){
		baud_rate = ATMEGA2560_BAUD_RATE;
		serial_ports[0] = ttyACM0;
		serial_ports[1] = ttyACM1;
		serial_ports[2] = ttyACM2;
		serial_ports[3] = ttyACM3;
		serial_ports[4] = ttyACM4;
		printf("# MCU selected: Atmega2560\n");
		return true;
	}
	else if( _mcu == ESP32 ){
		baud_rate = ESP32_BAUD_RATE;
		serial_ports[0] = ttyUSB0;
		serial_ports[1] = ttyUSB1;
		serial_ports[2] = ttyUSB2;
		serial_ports[3] = ttyUSB3;
		serial_ports[4] = ttyUSB4;
		printf("# MCU model: ESP32\n");
		return true;
	}
	else if( _mcu == ATMEGA328 ){
		printf("mcu not yet supported! exit...\n");
		return false;
	}
	else {
		printf("mcu non recognized! exit...\n");
		return false;
	}
}


/**
 * @brief find serial device and open
 * 
 * @param fd file descriptor pointer
 * @return ttyACM* device number, -1 if no device found
 */
int openSerialCommunication(int* fd){
	printf("\nSearching for MCU device on serial ports...\n");

	for (int i=4; i>-1 ; i--){
		*fd = open(serial_ports[i], O_RDWR | O_NOCTTY | O_NDELAY);
		if (*fd >= 0){
			printf("-> %s found.\n", serial_ports[i]);
			return i;
		}
		else { 
			printf("# %s not found\n", serial_ports[i]);
		}
	}
	return -1;
}


/**
 * @brief Set serial port attributes using termios structure
 * 
 * @param fd file descriptor
 */
void setSerialAttributes(int fd){
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings, baud_rate); /* Set Read Speed as 115200 	*/
	cfsetospeed(&SerialPortSettings, baud_rate); /* Set Write Speed as 115200    */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity*/
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits, here it is cleared so 1 Stop bit*/
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;		/*No Output Processing*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){
		printf("\n ERROR! cannot set serial attributes\n\n");
		exit(EXIT_FAILURE);
	}
	else{
		int baud_rate_value = 0;
		if( baud_rate == B9600 ) baud_rate_value = 9600;
		else if( baud_rate == B115200 ) baud_rate_value = 115200;
		printf("\n  | BaudRate = %d \n  | StopBits = 1 \n  | Parity   = none\n\n", baud_rate_value);
	}
}

/**
 * @brief Point_t* ==> uint8_t*
 * 	encode 16 bytes structure to 8 bytes buffer
 * @param config 16 bytes structure
 * @param buf 8 bytes buffer
 */
void encodeConfig(Point_t* ball_pos, uint8_t* buf){
	buf[0] = (ball_pos->x) & 0xFF;	//low bits
	buf[1] = (ball_pos->x) >> 8;		//high bits
	buf[2] = (ball_pos->y) & 0xFF;	//low bits
	buf[3] = (ball_pos->y) >> 8;		//high bits
	buf[4] = '\n';
}

/**
 * @brief Close serial communication
 * 
 * @param fd file descriptor
 * @param config servo config
 */
void closeSerialCommunication(int* fd, Point_t* ball){

	uint8_t buf[5];
	ball->x = X_HALF_ANGLE;
	ball->y = Y_HALF_ANGLE;
	memcpy(buf, ball, 4);

	int ret = write(*fd, (void*)buf, sizeof(buf));
	if( ret != (int)(sizeof(buf)/sizeof(uint8_t)) ){
		printf("\n  -- %d bytes exprected but %d found\n",
		(int)(sizeof(buf)/sizeof(uint8_t)), ret);
	}

	sleep(2); //required to make flush work
	tcflush(*fd, TCIOFLUSH);
	ret = close(*fd);
	if(ret != 0) {
		printf("\n  -- close(fd) syscall failed [%d]\n", ret);
		exit(EXIT_FAILURE);
	}
}
