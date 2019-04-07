/**
 * @file manual_mode.cpp
 * @author Giuseppe Sensolini
 * @brief MANUAL MODE
 * @version 1.1
 * @date 2019-02-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "modes.h"
#include "../utils.h"


//=============================================================================
//:::::::::::::: MANUAL :::::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
int manual_mode(){
    
    int fd = -1;
    int ret __attribute__((unused)); /*for unused variables suppression*/

//===== INITIALIZE SERIAL COMMUNICATION =======================================

	int device_opened = openSerialCommunication(&fd);
	if( device_opened >= 0 ){
		setSerialAttributes(fd);
		printf("# %s successfully opened\n", serialPorts[device_opened]);
	}
	else{
		perror("\nERROR: NO /dev/ttyACM* DEVICE CONNECTED");
		exit(EXIT_FAILURE);
	}

	int bytes_written = 0;
	uint8_t buf[5];
	memset(buf, 0, sizeof(buf));
	printf("# write_buffer allocated\n");

//===== SETUP DATA STRUCTURES =================================================

//_ Initialize servo config___________________________
	ServoConfig_t config = {
		.xPulse = X_HALF_ANGLE,
		.yPulse = Y_HALF_ANGLE
	};
	printServoConfig(config);
    
//=============================================================================

	char tmp;
	printf("\n # All parameters setted, ready to go...\n\
			\n -> Press enter to start <- \n");
	ret = scanf("%c", &tmp);

//_ Handshake with avr________________________________
	/*  [not a real handshake, this is needed to setup the avr)] */
	printf("# Handshake... ");
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	printf("Done. \n");
    return 0;

	char answ = 0;
    char yes[2] = {'y','Y'}, no[2] = {'n','N'};

//======= L O O P =============================================================
    while(true){

		printf("\nDo you want to move servos? (y/n)\n");
		ret = scanf("%c", &answ);

		if( answ==yes[0] || answ==yes[1] ){

			printf("\nxPulse: ");
			ret = scanf(" %d", &config.xPulse);

            printf("yPulse: ");
			ret = scanf(" %d", &config.yPulse);

            //encode and send to avr
			encodeConfig(&config, buf);	//Create Packet
			bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
			if(bytes_written != 5){
				perror("Error: write() syscall failed");
				exit(EXIT_FAILURE);
			}
        }
        else if( answ==no[0] || answ==no[1] ){
            break;
        }
    }
//=============================================================================

//_ close serial_______________________________________
    printf("\n========== EXIT ROUTINE ==========\n\n");
	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	printf("\n==================================\n\n");

    return 0;
}

