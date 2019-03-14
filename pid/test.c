#include <stdlib.h>
#include <stdio.h>

#include "pid.h"

/* derivative filter
*       TESTING UNIT: average low pass filtering
*
*/

int main(){//////////////////////////////////////////////////////////////////////////

    printf("\n# creating PID structs... ");
    PID_t XPID = createPID(30, 15, 5, 240, false);
    printf("Done.\n");

    printf("\n# LOW AVERAGE FILTER TEST:\n");
    int T = 100;
    short x;
    uint16_t pos[8] = { 1,1,1,1,1,1,1,1 };

    x = smoothingFilter(pos, T);
    printf("pos = {1,1,1,1,1,1,1,1}\nresult %d\n\n", x);
    
    pos[0] = 10;
    x = smoothingFilter(pos, T);
    x = saturationFilter(x, -30, 30);
    printf("pos = {10,1,1,1,1,1,1,1}\nresult %d\n\n", x);

    pos[0] = 100;
    x = smoothingFilter(pos, T);
    x = saturationFilter(x, -50, 50);
    printf("pos = {100,1,1,1,1,1,1,1}\nresult %d\n\n", x);

    pos[1] = 60;
    pos[2] = 0;
    pos[3] = 0;
    x = smoothingFilter(pos, T);
    x = saturationFilter(x, -50, 50);
    printf("pos = {100,60,0,0,1,1,1,1}\nresult %d\n\n", x);
        



    return 0;
}











//