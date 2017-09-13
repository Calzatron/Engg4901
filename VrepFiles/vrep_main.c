#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "extApi.h"
#include <string.h>

typedef struct info {

    int objectCount;
    int* objectHandles;
    char** objectNames;
} info;


info* makeInfo(void);

int main(int argc, char* argv[]){
printf("getting client\n");
    simxFinish(-1);
    int clientID = simxStart((simxChar*)"10.89.198.90",19999,true,true,5000,5);
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
            info_ptr->objectNames = malloc(sizeof(char*)*info_ptr->objectCount);
            info_ptr->objectHandles = objectHandles;
            for (int x = 0; x < info_ptr->objectCount; x++){
                printf("Handle %d\n", info_ptr->objectHandles[x]);
            }
                //int stringCount;
                //simxChar* string;
                printf("here\n");
            simxChar* string2;

            int handlesCount;
            int* handles;
            int stringCount;
            simxChar* stringData;
// simx_opmode_oneshot_wait
            simxGetObjectGroupData(clientID,sim_appobj_object_type,0,
                &handlesCount,&handles,NULL,NULL,NULL,NULL,&stringCount,
                &stringData,simx_opmode_blocking);

            simxGetObjectGroupData(clientID,sim_appobj_object_type,0,
                &handlesCount,&handles,NULL,NULL,NULL,NULL,&stringCount,
                &string2,simx_opmode_blocking);



                //int ret = simxGetObjectGroupData(clientID, sim_object_joint_type, 0, NULL, NULL, NULL, NULL,
                //        NULL, NULL, NULL, NULL, simx_opmode_blocking);
                printf("returned\n");
                printf("returned string: %s %d %s\n", stringData, stringCount, string2);
                printf("complete\n");
            for(int i = 0; i < info_ptr->objectCount; i++){
                simxChar* replyData;
                int replySize[1] = {1};
                int bufferSize = 0;
                int replySize1 = 1;
                simxChar* buffer;
                simxInt num = i;
                ret = simxCallScriptFunction(clientID, "", sim_scripttype_mainscript,
                        "get_object_names", 1, &num, 0, NULL, 0, NULL, 0, NULL, 
                        NULL, NULL, NULL, NULL, &replySize, &replyData, NULL, NULL,
                        simx_opmode_blocking);
                //printf("lll");fflush(stdout); 
          /*      ret = simxQuery(clientID, "request", "send me the collidable handles", 256,
                        "reply", &replyData, &replySize, 5000);
          */    //printf("this is data: %s\n", replyData);
                info_ptr->objectNames[i] = malloc(sizeof(char)*(strlen(replyData)+1i));
                strcpy(info_ptr->objectNames[i], replyData);
            }
            printf("finidhed\n");
            printf("aa: %s\n", info_ptr->objectNames[40]);
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

