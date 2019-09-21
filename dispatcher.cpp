/**
 * @file main.cpp
 * @author Giuseppe Sensolini
 * 
 * @brief 		BALL BALANCING PID SYSTEM
 * @repository 	https://github.com/JiuSenso/Ball-Balancing-PID-System.git
  
  
  		after compiling it(see README.md for details)
  		can be launched with different option flags:
  		
		[1] "./run"
 			standard mode: better performance, minimal GUI.

		[2]	"./run -s"
			setting mode: set pid gains and computer vision parameters.
 
		[3]	"./run -d"
  			debug moded: start debug mode: a better GUI and print utilities,
 			little bit slower than standard mode.
  
		[4]	"./run -m"
		  	manual mode: platform can be controlled directly from with joypad.
  
  		*note: one and only one flag can be used.
 			
 * @version 1.3
 * @date 2019-03-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <getopt.h>
#include "modes/modes.h"

/**
 * @brief MAIN works like a dispatcher
 */
int main(int argc, char* argv[]){
	int opt, ret;
	bool flags_enabled = false;

  	while((opt = getopt (argc, argv, "fsdm")) != -1){
		flags_enabled = true;

    	switch (opt){
			case 'f':
				printf("# Forcing AVR reboot... \n\n");
				ret = system("cd avr && bash upload.sh");
				break;
	
			case 's':
				settings_mode();
				break;
				
			case 'd':
				printf("Not yet implemented! This mode is avaible only for Version 1.x")
				//debug_mode();
				break;

			case 'm':
				printf("Not yet implemented! This mode is avaible only for Version 1.x")
				//manual_mode();
				break; 		
			
			case '?':
				printf("# Avaible flags:\
				\n    -f : force avr reboot\
				\n    -s : open setting window\
				\n    -d : enable debug mode\
				\n    -m : enable manual mode\n");
				exit(EXIT_FAILURE);		
		}
	}

	if (!flags_enabled) standard_mode();
	
	return 0;
}