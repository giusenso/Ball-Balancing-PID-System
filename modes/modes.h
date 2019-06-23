#ifndef MODES_H
#define MODES_H

#include <stdlib.h>
#include <unistd.h>
#include "../ball_tracker/ball_tracker.h"	/* 	computer vision and GUI			*/
#include "../serial/serial.h"				/* 	serial communication			*/
#include "../settings/file_handler.h"		/* 	pid and hsv settings 	        */

int standard_mode();

int debug_mode();

int settings_mode();

int manual_mode();



#endif
