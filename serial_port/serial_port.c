
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
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port.h"


	//initialize serial communication______________
bool openSerialCommunication(int* fd){

	const char* serialPorts[5]= {	ttyACM0,
									ttyACM1,
									ttyACM2,
									ttyACM3,
									ttyACM4	
								};

	for (int k=0 ; k<5 ; k++){
		*fd = open(serialPorts[k], O_RDWR | O_NOCTTY | O_NDELAY);
		if (*fd >= 0){
			printf("# %s successfully opened\n", serialPorts[k]);
			return true;
		}
	}
	if (*fd < 0){
		printf("\nERROR: no serial device avaible!\n");
		return false;
	}
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
		printf("\n  BaudRate = %d \n  StopBits = 1 \n  Parity   = none", BAUD_RATE);
	}
}



//_________SERVOCONFIG_T* ===> UINT8_T*	[run on pc]
void encodeConfig(ServoConfig_t* config, uint8_t* buf){
	buf[0] = config->ServoX;
	buf[1] = config->ServoY;
	buf[2] = config->flag1;
	buf[3] = config->flag2;
}

//_________UINT8_T* ===> SERVOCONFIG_T*	[run on arduino]
void decodeConfig(uint8_t* buf, ServoConfig_t* config){
	config->ServoX = buf[0];
	config->ServoY = buf[1];
	config->flag1 =  buf[2];
	config->flag2 =  buf[3];
}



void printServoConfig(ServoConfig_t* config){
	printf("\n	 ============ ServoConfig(%d) ============= \n", (int)sizeof(ServoConfig_t));
	printf("	||	ServoX:		%d	0x%02X	 ||\n", config->ServoX, config->ServoX);
	printf("	||	ServoY:		%d	0x%02X	 ||\n", config->ServoY, config->ServoY);
	printf("	||	flag1:		%d	0x%02X	 ||\n", config->flag1, config->flag1);
	printf("	||	flag2:		%d	0x%02X	 ||\n", config->flag2, config->flag2);
	printf("	 ========================================= \n");
}

void printEncodedPack(uint8_t* buf){
	
	printf("\n	 ======== ======== ======== ========\n	|");

	for (int i=0 ; i<sizeof(ServoConfig_t) ; i++){
		printf("  0x%02X  |", buf[i]);
	}
	printf("\n	 ======== ======== ======== ========\n");
}



