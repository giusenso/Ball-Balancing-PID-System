
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
	ServoConfig_t config = {
		.servoX = 0x6162,
		.servoY = 0x6364
	};

	uint8_t buf[6] = { 0,0,0,0,0,0 };

	printf("\nLook at cutecom, now!\n");
	usleep(2000000); // wait 2 seconds

	while(1){
		printServoConfig(&config);
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, 6);
		printf("\n# %d Bytes written to /dev/ttyACM device\n\n", bytes_written);

		usleep(4000000);
	}

    return 0;

}
