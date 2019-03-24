
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

const char* serialPorts[5]= {	ttyACM0,
								ttyACM1,
								ttyACM2,
								ttyACM3,
								ttyACM4
							};

//--------------------------------------------------------
//initialize serial communication______________
int openSerialCommunication(int* fd){
	printf("\nSearch for AVR device on serial ports...\n");
	int k = 0;
	for ( ; k<5 ; k++){
		*fd = open(serialPorts[k], O_RDWR | O_NOCTTY | O_NDELAY);
		if (*fd >= 0){
			printf(" /dev/ttyACM%d found.\n", k);
			return k;
		}
		else { 
			printf("# /dev/ttyACM%d not found\n", k);
		}
	}
	return -1;
}

//-------------------------------------------------------
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
		printf("\n  | BaudRate = %d \n  | StopBits = 1 \n  | Parity   = none\n\n", BAUD_RATE);
	}
}

//-------------------------------------------------------
void closeSerialCommunication(int* fd, ServoConfig_t* config){

	uint8_t buf[5];
	config->servoX = X_HALF_ANGLE;
	config->servoY = Y_HALF_ANGLE;
	encodeConfig(config, buf);

	int ret = write(*fd, (void*)buf, sizeof(buf));
	if( ret != (int)(sizeof(buf)/sizeof(uint8_t)) ){
		printf("\n  -- %d bytes exprected but %d found\n",
		(int)(sizeof(buf)/sizeof(uint8_t)), ret);
	}

	sleep(2); //required to make flush work, for some reason
	tcflush(*fd, TCIOFLUSH);

	ret = close(*fd);
	if(ret != 0) printf("\n  -- close(fd) syscall failed [%d]\n", ret);
}

//-------------------------------------------------------
// SERVOCONFIG_T* ===> UINT8_T*
void encodeConfig(ServoConfig_t* config, uint8_t* buf){
	buf[0] = (config->servoX) & 0xFF;	//low bits
	buf[1] = (config->servoX) >> 8;		//high bits
	buf[2] = (config->servoY) & 0xFF;	//low bits
	buf[3] = (config->servoY) >> 8;		//high bits
	buf[4] = '\n';
}
//-------------------------------------------------------
/*
bool handShake(int* fd){	//to be tested
	printf("\n HANDSHACKING: ");
	int i, j, bytes_written, bytes_read;
	uint8_t write_buf[5], read_buf[5];

	for (i=1; i<6; i++){
		for(j=0; j<4; j++) write_buf[j] = 7*j*i+7;
		write_buf[4] = '\n';

		bytes_written = write(*fd, write_buf, sizeof(write_buf));
		usleep(150000);
		printf("=");

		bytes_read = read(*fd, read_buf, sizeof(read_buf));
		usleep(150000);
		printf("=");

		for(j=0; j<5; j++) if(write_buf[j] != read_buf[j]) return false;
	}
	bytes_written = write(*fd, (void*)"done\n", sizeof(write_buf));
	printf(" Done.\n\n");
	usleep(250000);
	return true;
}
*/
//-------------------------------------------------------
void printServoConfig(ServoConfig_t config){
	printf("\n   ======================================\n");
	printf("  |  servoX: %d   ||   servoY: %d  |\n", config.servoX, config.servoY);
	printf("   ======================================\n");
}
//-------------------------------------------------------
void printEncodedPack(uint8_t* buf){
	printf("\n   =========================================\n  |");
	printf(" 0x%02X  |", buf[0]);
	printf(" 0x%02X  ||", buf[1]);
	printf(" 0x%02X  |", buf[2]);
	printf(" 0x%02X  ||", buf[3]);
	printf(" 0x%02X  |", buf[4]);
	printf("\n   =========================================\n");
}
