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

	if(openSerialCommunication(&fd) < 0){
		printf("ERROR: cannot open serial communication. exit.\n");
		return -1;
	}
	else printf("Serial communication estabilished\n");

	setSerialAttributes(fd);

	/*
	if(!handShake(&fd)) {
		printf("ERROR: HANDSHAKE FAILED. exit.\n");
		return -1;
	}
	else printf("Handshake successfully done\n");
	*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Initialize data structure________________
	ServoConfig_t config;

	uint8_t buf[5] = { 0,0,0,0,0 };

	printf("\nStarting...\n");
	usleep(4000000); // wait 2 seconds

	int count = 0, speed;
	int refresh = (1/15.0)*1000000;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 1: X UP AND DOWN MOTION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	config.servoX = MIN_ANGLE;
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	usleep(999000);
	speed = 400;
	while(config.servoX<MAX_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(refresh); count++;

		config.servoX += speed;
	}
	speed = 400;
	while(config.servoX>MIN_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(refresh); count++;
		config.servoX -= speed;
	}
	usleep(500000);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 2: Y UP AND DOWN MOTION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	config.servoY = MIN_ANGLE;
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	usleep(999000);

	speed = 400;
	while(config.servoY<MAX_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(refresh); count++;

		config.servoY += speed;
	}
	speed = 400;
	while(config.servoY>MIN_ANGLE){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		usleep(refresh); count++;

		config.servoY -= speed;
	}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 3: X AND Y OPPOSITE MOTION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	printf("\n# TEST 3: OPPOSITE MOTION\n");
	usleep(2000000);

	config.servoX = MAX_ANGLE;
	config.servoY = MIN_ANGLE;
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	usleep(999000);

	for(int cc=0 ; cc<3 ; cc+=1){
		speed = 400;
		while(config.servoX>MIN_ANGLE || config.servoY<MAX_ANGLE ){
			encodeConfig(&config, buf);
			printEncodedPack(buf);

			bytes_written = write(fd,(void*)buf, sizeof(buf));
			printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
			printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

			config.servoX -= speed;
			config.servoY += speed;

			count++;
			usleep(refresh);
		}
		while(config.servoX<MAX_ANGLE || config.servoY>MIN_ANGLE ){
			encodeConfig(&config, buf);
			printEncodedPack(buf);

			bytes_written = write(fd,(void*)buf, sizeof(buf));
			printf("\n	Speed: %d\nX pulse: %d |	Y pulse: %d\n", speed, config.servoX, config.servoY);
			printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

			config.servoX += speed;
			config.servoY -= speed;

			count++;
			usleep(refresh);
		}
		speed+=100;
	}
	usleep(1000000);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST 4: ARMONIC MOTION, PLATFORM CIRCOLAR MOTION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	printf("\n# TEST 4: ARMONIC MOTION\n");
	usleep(2000000);

	count = 0;
	speed = 400;
	config.servoX = MAX_ANGLE;
	config.servoY = MIN_ANGLE;
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	float q = ((float)config.servoX - HALF_ANGLE)/(ANGLE_OFFSET/80);
	usleep(999000);

	while(count<350){
		encodeConfig(&config, buf);
		printEncodedPack(buf);

		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("\n	X pulse: %d |	Y pulse: %d\n", config.servoX, config.servoY);
		printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n", count, bytes_written);

		q = ((float)config.servoX )/((float)ANGLE_OFFSET/80);
		config.servoX -= count/2 * sin(q*(PI/180));

		q = ((float)config.servoY )/((float)ANGLE_OFFSET/80);
		config.servoY += count/2 * cos(q*(PI/180));

		count++;
		usleep(refresh);
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
		config.servoY -= count/2 * sin(q*(PI/180));

		count++;
		usleep(refresh);
	}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    return 0;
}
