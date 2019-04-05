/**
 * @file main.cpp
 * @author Giuseppe Sensolini
 * 
 * @brief 		BALL BALANCING PID SYSTEM
 * @repository 	https://github.com/JiuSenso/Ball-Balancing-PID-System.git
  
  
  		after compiling it(see README.md for details)
  		can be launched with different option flags:
  		
		[1] "./run -s"
 			standard mode: better performance but minimal GUI

		[2]	"./run -settings"
			setting mode: set pid gains and computer vision parameters
 
		[3]	"./run -debug"
  			debug moded: start debug mode: a better GUI and print utilities,
 			little bit slower than standard mode
  
		[4]	"./run -manual"
		  	manual mode: platform can be controlled directly from terminal.
  
  		*note: one and only one flag can be used
 			
 * @version 1.3
 * @date 2019-03-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "modes/modes.h"

void printFlagOptions();

int main(int argc, char* argv[]){

	if (strcmp(argv[1],"-s") == 0){
		standard_mode();
		exit(EXIT_SUCCESS);
	}
	if (strcmp(argv[1],"-settings") == 0){
		settings_mode();
		exit(EXIT_SUCCESS);
	}
	if (strcmp(argv[1],"-debug") == 0){
		debug_mode();
		exit(EXIT_SUCCESS);
	}
	if (strcmp(argv[1],"-manual") == 0){
		manual_mode();
		exit(EXIT_SUCCESS);
	}	
	
	else{
		printf("error: INVALID FLAG!\n");
		printFlagOptions();
	}
	
	return 0;
}



/**
 * @brief 
 * 
 */
void printFlagOptions(){
    printf("Use one and only one flag to enable one of this modes:\n\n");
    printf(" [1] STANDARD MODE: -s\n [2] SETTINGS MODE: -settings\n\
 [3] DEBUG MODE:    -debug\n [4] MANUAL MODE:   -manual\n\n");
}