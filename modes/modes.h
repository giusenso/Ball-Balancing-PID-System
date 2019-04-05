#ifndef MODES_H
#define MODES_H

#include "../ball_tracker/ball_tracker.h"	/* 	computer vision and GUI			*/
#include "../pid/pid.h"					    /* 	pid implementation and filters	*/
#include "../serial/serial.h"				/* 	serial communication			*/
#include "../settings/file_handler.h"		/* 	pid and hsv settings 	        */
#include <time.h>

int standard_mode();

int debug_mode();

int settings_mode();

int manual_mode();



#endif
