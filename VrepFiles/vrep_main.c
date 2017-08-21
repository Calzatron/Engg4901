#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "extApi.h"


typedef struct info {

    int objectCount;
    int* objectHandles;

} info;


info* makeInfo(void);

int main(int argc, char* argv[]){
printf("getting client\n");
    simxFinish(-1);
    int clientID = simxStart((simxChar*)"10.89.197.136",19999,true,true,5000,5);
printf("got client\n");
    if (clientID != -1) {
        info* info_ptr = makeInfo();
        printf("Successfully connected to Vrep\n");
        // retrieve data in a blocking fashion - ensure response from vrep
        int objectCount;        // integer variable
        int* objectHandles;     //array of ints of unknown size
        
        int ret = simxGetObjects(clientID, sim_handle_all, &objectCount, 
                &objectHandles, simx_opmode_blocking);
        if (ret == simx_return_ok){
            printf("Number of objects in scene: %d \n", objectCount);
            /*  store all object handles  */;
            info_ptr->objectCount = objectCount;
            info_ptr->objectHandles = malloc(sizeof(int)*info_ptr->objectCount);
            info_ptr->objectHandles = objectHandles;
            for (int x = 0; x < info_ptr->objectCount; x++){
                printf("Handle %d\n", info_ptr->objectHandles[x]);
            }
        } else {
            printf("Remote API function call returned with error: %d\n", ret);
        }

        extApi_sleepMs(2000);

        simxFinish(-1); // close all open connections
    } else {




        printf("Connection failed %d\n", clientID);
        return 1;
    }
    return 0;
}

info* makeInfo(void){
    /* Initialize a struct to store all the Vrep scene info */
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}


