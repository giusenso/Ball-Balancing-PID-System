
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions			*/
#include <termios.h> /* POSIX Terminal Control Definitions	*/
#include <unistd.h>  /* UNIX Standard Definitions			*/ 
#include <errno.h>   /* ERROR Number Definitions			*/

#include "serial_port.h"

int main(){

	int fd;
	int bytes_written;

	if (openSerialCommunication(&fd) < 0){
		printf("ERROR: cannot open serial communication\n");
		return -1;	
	}
	else printf("Serial communication estabilished\n");

	setSerialAttributes(fd);

	//Initialize data structure________________
	ServoConfig_t* config = (ServoConfig_t*) malloc(sizeof(ServoConfig_t));
	config->servoX = 0x6622;
	config->servoY = 0x3344;

	uint8_t* buf = (uint8_t*)malloc(sizeof(ServoConfig_t));

	printf("\nLook at cutecom, now!\n");
	usleep(2000000); // wait 2 seconds
	
	while(1){

		encodeConfig(config, buf);
		printf("\nSending pack...");
		printEncodedPack(buf);

		bytes_written = write(fd, buf, 32);
		printf("\ndone. %d Bytes written to /dev/ttyACM device\n\n", bytes_written);

		usleep(2000000);
	}

	//exit routine__________________
	free(config);
	free(buf);
	printf("\nData structures deallocated... exit.\n");
	
    return 0;

}