#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "extApi.h"
#include <string.h>

typedef struct info {

    int clientID;
    int objectCount;
    int* objectHandles;
    int* isJoint;
    char** objectNames;
    
    char* response;

} info;

info* makeInfo(void);
void get_object_names_vrep(info* info_ptr);
void write_object_info(info* info_ptr, char* filename);
void get_command(info* info_ptr);


int main(int argc, char** argv){
    
    printf("getting client\n");
    simxFinish(-1);
    info* info_ptr = makeInfo();
    info_ptr->response = malloc(sizeof(char)*128);
    //while(1){
    //}
    info_ptr->clientID = simxStart((simxChar*)"192.168.0.16",19999,true,true,5000,5);
    printf("got client\n");
    if (info_ptr->clientID != -1) {
        printf("Successfully connected to Vrep\n");
        // retrieve data in a blocking fashion - ensure response from vrep
        int objectCount;        // integer variable
        int* objectHandles;     //array of ints of unknown size
        
        int ret = simxGetObjects(info_ptr->clientID, sim_handle_all, &objectCount, 
                &objectHandles, simx_opmode_blocking);
        if (ret == simx_return_ok){
            printf("Number of objects in scene: %d \n", objectCount);
            /*  store all object handles  */;
            info_ptr->objectCount = objectCount;
            info_ptr->objectHandles = malloc(sizeof(int)*info_ptr->objectCount);
            info_ptr->isJoint = malloc(sizeof(int)*info_ptr->objectCount);
            info_ptr->objectHandles = objectHandles;
            printf(">> %d\n", info_ptr->objectHandles[40]);
            /*  retrieve all object names from custom function in VRep Main */

            get_object_names_vrep(info_ptr);

            /*  Print a sample name to check */
            printf("aa: %s\n", info_ptr->objectNames[40]);

            /*  write information to a file */
            if (argc == 2){
                /* filename specified */
                write_object_info(info_ptr, argv[1]);
            } else if (argc == 1){
                write_object_info(info_ptr, "objects");   
            }
            printf("Written to file\n");
            get_command(info_ptr);
        } else {
            printf("Remote API function call returned with error: %d\n", ret);
        }

        extApi_sleepMs(2000);

        simxFinish(-1); // close all open connections
    } else {

        printf("Connection failed %d\n", info_ptr->clientID);
        return 1;
    }
    return 0;
}

info* makeInfo(void){
    /* Initialize a struct to store all the Vrep scene info */
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}


void get_command(info* info_ptr){
    
    printf(">> ");
    int c;
    int i = 0;;
    
    char response[128];
    while((c = getchar()) != '\n'){
        response[i] = c;
        ++i;
    }
    response[i] = '\0';
    strcpy(info_ptr->response, response);
    printf("received: %s\n", info_ptr->response);
}


void get_object_names_vrep(info* info_ptr){
    /*  Retrieves object names for corresponding object handle,
     *  stored in info struct
     *
     *  Calls a custom function in VRep main script, which takes in an
     *  object handle and returns the object's name.
    */
    int replySize[1] = {1};
    info_ptr->objectNames = malloc(sizeof(char*)*info_ptr->objectCount);

    for(int i = 0; i < info_ptr->objectCount; i++){
        int ret = 0;
        simxChar* replyData;
        simxInt num = i;
        info_ptr->isJoint[i] = 0;
        ret = simxCallScriptFunction(info_ptr->clientID, "", sim_scripttype_mainscript,
                "get_object_names", 1, &num, 0, NULL, 0, NULL, 0, NULL, 
                NULL, NULL, NULL, NULL, &replySize, &replyData, NULL, NULL,
                simx_opmode_blocking);
        if (ret != simx_return_ok){
            printf("ret not ok\n");
        }
        info_ptr->objectNames[i] = malloc(sizeof(char)*(strlen(replyData)+1));
        strcpy(info_ptr->objectNames[i], replyData);
        char* underScore = "_";
        char* name = malloc(sizeof(char)*(strlen(info_ptr->objectNames[i])+1));
        strcpy(name, info_ptr->objectNames[i]);
        char* token = strtok(name, underScore);
        while (token != NULL){
            char joint[5] = {'j', 'o', 'i', 'n', 't'};
            char motor[5] = {'m', 'o', 't', 'o', 'r'};
            int count = 0;
            for (int c = 0; c < 5; c++){
                if ((token[c] == joint[c]) || (token[c] == motor[c])){   
                    ++count;
                }
            }
            token = strtok(NULL, underScore);
            if (count == 5){
                info_ptr->isJoint[i] = 1;
            }
        }
    }
    printf("Got object names\n");
}


void write_object_info(info* info_ptr, char* filename){

    /*  Writes all the object handles and corresponding names to filename
     *  if filename already exists, it clears the file before writing
     *  filename is either an input argument or by default called "objects"
     */

    FILE* object_fp = fopen(filename, "w+");
    if (object_fp == NULL){
        fprintf(stdout, "Failed to generate file\n");
        exit(1);
    }
    
    for (int i = 0; i < info_ptr->objectCount; i++){
        if (info_ptr->isJoint[i]){ 
            char line[256];
            sprintf(line, "%d %s\n", i, info_ptr->objectNames[i]);
            fputs(line, object_fp);
        }

    }
    fflush(object_fp);
    fclose(object_fp);

}










