#include <time.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "short_linked_list.h"

const int N = 100000000;
clock_t start1, end1, start2, end2;

int main(){
//// Testing Linked list /////////////////////////////////////
    printf("\n# linked list test:\n");
    ListHead head;
    List_init(&head, 10, 25);

    short avg = List_update(&head, 0);
    List_update(&head, 200);
    List_update(&head, 29);

    start1 = clock();
    for(int i=0 ; i<N ; i++){
        avg = List_update(&head, rand()%299+1);
    }
    end1 = clock();
    List_print(&head);
    printf("avg = %d\n", avg );
    printf("TIME: %lf\n", (float)(end1 - start1)/CLOCKS_PER_SEC);

//// Testing Array //////////////////////////////////////////
    printf("\n# array test:\n");
    short sum = 450;
    const short sz = 10;
    short buf[10] = { 100,50,25,25,25,100,50,25,25,25 };

    start2 = clock();
    for(int i=0 ; i<N ; i++){
        sum -= buf[9];
        buf[9] = buf[8];
        buf[8] = buf[7];
        buf[7] = buf[6];
        buf[6] = buf[5];
        buf[5] = buf[4];
        buf[4] = buf[3];
        buf[3] = buf[2];
        buf[2] = buf[1];
        buf[1] = buf[0];
        buf[0] = rand()%299+1;
        sum += buf[0];
        avg = sum/sz;
    }
    end2 = clock();
    printf("avg = %d\n", avg );
    printf("TIME: %lf\n", (float)(end2 - start2)/CLOCKS_PER_SEC);

//////////////////////////////////////////
    return 0;
}

