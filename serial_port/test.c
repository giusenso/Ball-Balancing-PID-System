
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port.h"


int main(){

	//Initialize data structure________________
	ServoConfig_t* config = (ServoConfig_t*) malloc(sizeof(ServoConfig_t));
		config->ServoX =	 0xAA;
		config->ServoY =	 0xBB;
		config->flag1 =		 0xCC;
		config->flag2 =		 0x00;
	

	ServoConfig_t* config2 = (ServoConfig_t*) malloc(sizeof(ServoConfig_t));
		config2->ServoX =	 0xFA;
		config2->ServoY =	 0xFB;
		config2->flag1 =     22;
		config2->flag2 =	 0xF0;

	char buf[ sizeof(ServoConfig_t)/sizeof(uint8_t) ];

	int fd;
	int bytes_written;

	if (!openSerialCommunication(&fd)){
		printf("\n ERROR: cannot open serial communication\n");
		return -1;	
	}
	else printf("Serial communication estabilished\n");

	setSerialAttributes(fd);

	printf("\nLook at cutecom, now!\n");
	usleep(2500000); // wait 2,5 seconds
		
	encodeConfig(config, buf);
	printf("\nSending pack...");
	printEncodedPack(buf);

	bytes_written = write(fd, buf, sizeof(ServoConfig_t));
	printf("done. %d Bytes written to /dev/ttyACM device\n\n", bytes_written);

	
	usleep(1000000); // wait 1 second
	encodeConfig(config2, buf);
	printf("\nSending pack...");
	printEncodedPack(buf);

	bytes_written = write(fd, buf, sizeof(ServoConfig_t));
	printf("done. %d Bytes written to /dev/ttyACM device\n\n", bytes_written);


	//exit routine__________________
	free(config);
	free(config2);
	//free(buf);
	printf("\nData structures deallocated... exit.\n");
	
    return 0;

}