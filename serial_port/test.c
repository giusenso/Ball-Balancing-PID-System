
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
		.servoX = 7000,
		.servoY = 37000
	};

	uint8_t buf[6] = { 0,0,0,0,0,0 };

	printf("\nLook at cutecom, now!\n");
	usleep(1500000); // wait 2 seconds

	int count = 0;
	while(config.servoX<38000 && config.servoY>7000){

		printServoConfig(&config);
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		printf("\nX pulse: %d", config.servoX);
		printf("\nY pulse: %d", config.servoY);
		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ==============================================\n", count, bytes_written);

		count++;
		config.servoX += 100;
		config.servoY -= 100;

		usleep(30000);
	}
	while(config.servoY<38000 && config.servoX>7000){

		printServoConfig(&config);
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		printf("\nX pulse: %d", config.servoX);
		printf("\nY pulse: %d", config.servoY);
		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ==============================================\n", count, bytes_written);

		count++;
		config.servoX -= 100;
		config.servoY += 100;

		usleep(30000);
	}

    return 0;

}
