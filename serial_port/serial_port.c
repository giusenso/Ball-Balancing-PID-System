
	/*====================================================================================================*/
	/* Sellecting the Serial port Number on Linux                                                         */
	/* ---------------------------------------------------------------------------------------------------*/
	/* /dev/ttyACM0 is the default serial        														  */
    /*====================================================================================================*/

	/*-------------------------------------------------------------*/
    	/* termios structure -  /usr/include/asm-generic/termbits.h    */
	/* use "man termios" to get more info about  termios structure */
	/*-------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port.h"

const char* serialPorts[5]= {	ttyACM0,
								ttyACM1,
								ttyACM2,
								ttyACM3,
								ttyACM4
							};
// ********************************************************************************


//initialize serial communication______________
int openSerialCommunication(int* fd){
	int k = 0;
	for ( ; k<5 ; k++){
		printf("/dev/ttyACM%d\n", k);
		*fd = open(serialPorts[k], O_RDWR | O_NOCTTY | O_NDELAY);
		if (*fd >= 0){
			return k;
		}
	}
	return -1;
}


// Setting the Attributes of the serial port using termios structure
void setSerialAttributes(int fd){
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){
		printf("\n  ERROR ! in Setting attributes");
	}
	else{
		printf("\n  BaudRate = %d \n  StopBits = 1 \n  Parity   = none\n\n", BAUD_RATE);
	}
}


//_________SERVOCONFIG_T* ===> UINT8_T*	[run on pc]
inline void encodeConfig(ServoConfig_t* config, uint8_t* buf){
	buf[0] = (config->servoX) & 0xFF;	//low bits
	buf[1] = (config->servoX) >> 8;		//high bits
	buf[2] = '\r';
	buf[3] = (config->servoY) & 0xFF;	//low bits
	buf[4] = (config->servoY) >> 8;		//high bits
	buf[5] = '\r';
}

//_________UINT8_T* ===> SERVOCONFIG_T*	[run on avr]
inline void decodeConfig(uint8_t* buf, ServoConfig_t* config){
	config->servoX = ((uint16_t*)buf)[0];
	config->servoY = ((uint16_t*)buf)[2];
}

void printServoConfig(ServoConfig_t* config){
	printf("\n	 ============ ServoConfig(%d) ============= \n", (int)sizeof(ServoConfig_t));
	printf("	||	servoX:		%d	0x%02X	 ||\n", config->servoX, config->servoX);
	printf("	||	servoY:		%d	0x%02X	 ||\n", config->servoY, config->servoY);
	printf("	 ========================================= \n");
}

void printEncodedPack(uint8_t* buf){
	printf("\n	 ===============================================\n	|");
	printf(" 0x%02X  |", buf[0]);
	printf(" 0x%02X  |", buf[1]);
	printf(" 0x%02X  |", buf[2]);
	printf(" 0x%02X  |", buf[3]);
	printf(" 0x%02X  |", buf[4]);
	printf(" 0x%02X  |", buf[5]);
	printf("\n	 ===============================================\n");
}
