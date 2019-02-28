#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions			*/
#include <termios.h> /* POSIX Terminal Control Definitions	*/
#include <unistd.h>  /* UNIX Standard Definitions			*/
#include <errno.h>   /* ERROR Number Definitions			*/
#include <math.h>

#include "serial_port.h"

int main(){

	int fd, bytes_written;

	if (openSerialCommunication(&fd) < 0){
		printf("ERROR: cannot open serial communication\n");
		return -1;
	}
	else printf("Serial communication estabilished\n");

	setSerialAttributes(fd);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Initialize data structure________________
	ServoConfig_t config;

	uint8_t buf[5] = { 0,0,0,0,0 };

	printf("\nStarting...\n");
	usleep(1500000); // wait 2 seconds

	int count = 0, speed;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 1: X UP AND DOWN MOTION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	config.servoX = MIN_ANGLE;
	speed = 150;
	while(config.servoX<MAX_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(30000); count++;
		
		config.servoX += speed;
	}
	speed = 250;
	while(config.servoX>MIN_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(30000); count++;
		
		config.servoX -= speed;
	}
	usleep(3000000);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 2: Y UP AND DOWN MOTION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	config.servoY = MIN_ANGLE;
	speed = 150;
	while(config.servoY<MAX_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(30000); count++;
		
		config.servoY += speed;
	}
	speed = 250;
	while(config.servoY>MIN_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(30000); count++;
		
		config.servoY -= speed;
	}
	usleep(3000000);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 3: X AND Y OPPOSITE MOTION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	config.servoX = MAX_ANGLE;
	config.servoY = MIN_ANGLE;
	speed = 150;
	while(config.servoX>MIN_ANGLE && config.servoY<MAX_ANGLE ){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		config.servoX -= speed;
		config.servoY += speed;

		count++;
		usleep(30000);
	}
	while(config.servoY<MAX_ANGLE && config.servoX>MIN_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	Speed: %d\nX pulse: %d |	Y pulse: %d\n", speed, config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		config.servoX += speed;
		config.servoY -= speed;

		count++;
		usleep(30000);
	}
	usleep(1000000);
	
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 4: ARMONIC MOTION, PLATFORM CIRCOLAR MOTION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	printf("\n ARMONIC TEST\n");
	usleep(2000000);
	
	count = 0;
	speed = 400;
	config.servoX = MAX_ANGLE;
	config.servoY = MIN_ANGLE;
	float q = ((float)config.servoX - HALF_ANGLE)/(ANGLE_OFFSET/80);
	
	while(count<350){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		q = ((float)config.servoX - HALF_ANGLE)/((float)ANGLE_OFFSET/80);
		config.servoX += count/2 * sin(q*(PI/180));

		q = ((float)config.servoY - HALF_ANGLE)/((float)ANGLE_OFFSET/80);
		config.servoY += count/2 * cos(q*(PI/180));

		count++;
		usleep(30000);
	}
	
	while(count<700){
		speed = 600;
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	Speed: %d\nX pulse: %d |	Y pulse: %d\n", speed, config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		q = ((float)config.servoX - HALF_ANGLE)/((float)ANGLE_OFFSET/80);
		config.servoX += count/2 * cos(q*(PI/180));

		q = ((float)config.servoY - HALF_ANGLE)/((float)ANGLE_OFFSET/80);
		config.servoY += count/2 * sin(q*(PI/180));

		count++;
		usleep(30000);
	}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    return 0;
}