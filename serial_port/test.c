
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

	int CENTER = 23200;
	int	MAX = CENTER+6000;
	int MIN	= CENTER-6000;
	
	//Initialize data structure________________
	ServoConfig_t config = { 
		.servoX = MIN,
		.servoY = MAX
	};

	uint8_t buf[6] = { 0,0,0,0,0,0 };

	printf("\nLook at cutecom, now!\n");
	usleep(1500000); // wait 2 seconds

	int count = 0;
	while(1){
		
		printServoConfig(&config);
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		printf("\nPULSE: %d", config.servoX);
		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ==============================================\n", count, bytes_written);

		count++;
		//config.servoX = MAX;
		//config.servoY = MIN;
		usleep(2300000);
	}
	
    return 0;

}