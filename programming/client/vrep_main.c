/*
*	Callum Rohweder
*	VREP client program
*
*/

/*	Include Files	*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <strsafe.h>
#include <windows.h> 
#include <tchar.h>
#include <string.h>
#include <math.h>
#include "extApi.h"
#include "project.h"
#include "joystick.h"
#include <process.h>
#include "Shlwapi.h"
#include <direct.h>
#include "kinematics.h"

/*	Global Definitions	*/
#define BUFSIZE 4096 

HANDLE  hConsoleOut;                 // Handle to the console   
HANDLE  hRunMutex;                   // "Keep Running" mutex   
HANDLE  hScreenMutex;                // "Screen update" mutex  
int     ThreadNr;                    // Number of threads started   
CONSOLE_SCREEN_BUFFER_INFO csbiInfo; // Console information 


/*	Function Definitions	*/
info* makeInfo(void); 
move* makeMove(void);
void get_object_names_vrep(info* info_ptr);
void write_object_info(info* info_ptr, char* filename);
void get_command(info* info_ptr, move* move_ptr);
void get_joint_angles_vrep(info* info_ptr, move* move_ptr);
void initial_arm_config_vrep(info* info_ptr, move* move_ptr);
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void interpret_command_fk(info* info_ptr, move* move_ptr);
void interpret_command_ik(info* info_ptr, move* move_ptr, bool commandLine);
void CreateChildProcess(void);
void WriteToPipe(void);
void ReadFromPipe(void);
void move_target_vrep(info* info_ptr, move* move_ptr, char direction, float duty);
void read_object_info(info* info_ptr, char* filename);
void get_position_vrep(info* info_ptr, simxFloat* position, int handle);
void get_orientation_vrep(info* info_ptr, simxFloat* orientation, int handle, int relativeHandle);


/*	Program Functions	*/

void initialise_program(info* info_ptr, char argc, char** argv) {
	/*	Determine the mode of the Jaco arm in Vrep from input commands	*/
	char inputBuffer[10];
	int i = 0;
	if (argc == 1) {
		printf("Usage: vrepClientProgram.exe fk/ik [object filename] [IP Address] [port]");
		printf("\nPlease specify a scene Kinematics mode (ik/fk)>> "); fflush(stdout);
		char c = getchar();
		
		while ((c != '\n') && (i < 10)) {
			/*	store input	*/
			inputBuffer[i] = c;
			c = getchar();
			++i;
		}
	}
	else {
		strcpy(inputBuffer, argv[1]);
		i = strlen(argv[1]) + 1;
	}
	/*	Search for mode in input buffer	*/
	info_ptr->programMode = malloc(sizeof(char) * 3);
	for (int j = 0; j < i; j++) {
		if (((inputBuffer[j] == 'i') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'I') && (inputBuffer[j + 1] == 'K'))) {
			strcpy(info_ptr->programMode, "ik");
		}
		else if (((inputBuffer[j] == 'f') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'F') && (inputBuffer[j + 1] == 'K'))) {
			strcpy(info_ptr->programMode, "fk");
		}
	}
	
	/*	Get IP Address and Port from input or use local host settings	*/
	char ipAddress[18];
	int port = 19999;
	if (argc > 3) {
		strcpy(ipAddress, argv[3]);
		char* ptr;
		port = strtol(argv[4], &ptr, 10);
	}
	else {
		strcpy(ipAddress, "127.0.0.1");
	}

	/*	Get ID for connection to VREP	*/
	info_ptr->clientID = simxStart((simxChar*)ipAddress, port, true, true, 5000, 5);
	printf("\nGot client at %s %d\n", ipAddress, port);

	/*	Check that the ID is valid	*/
	if (info_ptr->clientID != -1) {
		printf("Successfully connected to VREP\n");
	} else {
		printf("Connection failed %d\n", info_ptr->clientID);
		exit(1);
	}

	

}

int main(int argc, char** argv){
    
    printf("getting client\n");				// start
    simxFinish(-1);							// kill any existing coms to Vrep
    info* info_ptr = makeInfo();			// initialise structures
	move* move_ptr = makeMove();

	initialise_program(info_ptr, argc, argv);

    /*	Retrieve data in a blocking fashion - ensure response from vrep	*/
    int objectCount;					// integer variable
    int* objectHandles;					//array of ints of unknown size

    /*	Get the object handles from VREP for all objects in the scene	*/
    int ret = simxGetObjects(info_ptr->clientID, sim_handle_all, &objectCount, 
            &objectHandles, simx_opmode_blocking);

    /*	Check if succeeded	*/
	if (ret != simx_return_ok) {
		printf("Could not retrieve objects from VREP\n");
		printf("Remote API function call returned with error: %d\n", ret);
		return 1;
	}
    printf("Number of objects in scene: %d \n", objectCount);

    /*  store all object handles  */;
    /*	allocate space	*/
	info_ptr->objectCount = objectCount;
    info_ptr->objectHandles = malloc(sizeof(int)*info_ptr->objectCount);
    info_ptr->isJoint = malloc(sizeof(int)*info_ptr->objectCount);
	info_ptr->objectNames = malloc(sizeof(char*)*info_ptr->objectCount);

	for (int joint = 0; joint < objectCount; joint++) {
		/*	initially all objects aren't joints
		*	joint handles will be found later	*/
		info_ptr->isJoint[joint] = 0;
	}
	info_ptr->objectHandles = objectHandles;
    printf("DEBUG>> objectHandles[40] %d \n", info_ptr->objectHandles[40]);

    /*	Set-able boolean value to retrieve names from Vrep and store joints
	*	or just use known joint handles to identify joints	*/
	int en_name = 0;
	
	if (en_name) {
		/*	retrieve the object names from VREP and store them in the info struct	*/
		/*  calls custom function in VREP Main */
		get_object_names_vrep(info_ptr);
	}
	else if (argc >= 3) {
		/*	Check if need to fill in the file or if it is ready for reading		*/
		char objectFileName[20];
		strcpy(objectFileName, argv[2]);
		char* cwd = _getcwd(NULL, 0);
		char* objectFilePath = malloc(sizeof(char)*(strlen(cwd) + strlen(argv[2]) + 5));
		strcpy(objectFilePath, cwd);
		strcat(objectFilePath, "\\");
		strcat(objectFilePath, argv[2]);
		int exists = PathFileExists(objectFilePath);
		
		if (exists) {
			/*	read file for it's contents
			*	get joints etc	*/
			read_object_info(info_ptr, argv[2]);
		}
		else {
			/*	file didn't exist, get names from vrep and write them to the file
			*	only joints are stored in file of given filename
			*/
			get_object_names_vrep(info_ptr);
			write_object_info(info_ptr, argv[2]);
		}
		
		printf("File path: %s exists: %d true: %d\n", objectFilePath, exists, true);
	}
	else {
		/*	use hard-coded handles	*/
		for (int h = 0; h < info_ptr->objectCount; h++) {
			if ((h == 18) || (h == 21) || (h == 24) || (h == 27) || (h == 30) || (h == 33)) {
				info_ptr->isJoint[h] = 1;
				info_ptr->objectHandles[h] = h;
				printf("H = %d\n", h);
			} else {
				info_ptr->isJoint[h] = 0;
				info_ptr->objectHandles[h] = 0;
			}

			if (h == 127) {
				info_ptr->targetHandle = h;
			}

		}
	}

	/* Retrieve all angles of joints to be used in kinematics
	*	Store the positions in move_ptr->currAng[i] for the i'th joint */
	initial_arm_config_vrep(info_ptr, move_ptr);

	/* Create a child process to retreive button presses and joystick
	*	positions from the gaming controller	*/
	CreateChildProcess();
	printf("Child Process Created\n");

	/*	Begin a thread for processing the commands from either
	*	control inputs or the joystick	*/
	hConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	hScreenMutex = CreateMutex(NULL, FALSE, NULL);		// Cleared   
	hRunMutex = CreateMutex(NULL, TRUE, NULL);			// Set   
	ThreadNr = 0;										//initialise threadcount
	ThreadNr++;											// increment threadcount for command thread
	_beginthread(ReadFromPipe, 0, &ThreadNr);			// direct the process to their new home
	
	
	ThreadNr++;											// increment threadcount for command thread
	_beginthread(add_to_buffer, 0, &ThreadNr);			// direct the process to their new home
	
	

	while (1) {
		//printf(".");
		/*	Need to work out how to choose between command or joystick based control	*/
		if ((strcmp(info_ptr->programMode, "ik") == 0) && (joystickEnabled())) {

			//printf("ik and joystickEn\n"); fflush(stdout);
			/*	A joystick was found, get inputs from this	*/
			if (joystick_input_available()) {
				//printf("input_av\n");
				/*	A joystick command is available to action on	*/
				int* arr;
				arr = joystick_get_char();
				printf("arr:	%c %d \n", arr[0], arr[1]);

				info_ptr->response = malloc(sizeof(char) * 128);
				/*	the input is converted into the same format that command line ik
				*	inputs come in to allow joystick or command line inputs	*/
				sprintf(info_ptr->response, "%c %d", arr[0], arr[1]);
				printf("received: %s\n", info_ptr->response);
				interpret_command_ik(info_ptr, move_ptr, false);

			}
			

		}
		else {
			/*	Instead of getting input from joystick, get from command line input	*/
			printf("commandline ");
			get_command(info_ptr, move_ptr);
		}
	}


    extApi_sleepMs(2000);

    simxFinish(-1); // close all open connections

    return 0;
}


info* makeInfo(void){
    /* Initialize a struct to store all the Vrep scene info */
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}

move* makeMove(void) {
	/* Initialize a struct to store all the Vrep scene info */
	move* move_ptr = malloc(sizeof(move));
	return move_ptr;
}


void ShutDown(void) // Shut down threads   
{
	while (ThreadNr > 0)
	{
		// Tell thread to die and record its death.  
		ReleaseMutex(hRunMutex);
		ThreadNr--;
	}

	// Clean up display when done  
	WaitForSingleObject(hScreenMutex, INFINITE);
}


void get_command(info* info_ptr, move* move_ptr){
    /*	Prompts ready to receive command from stdin
	*	stores the input in the response buffer
	*	and sends it for interpreting	*/
    printf(">> ");
    int c;
    int i = 0;
    
    char response[128];
    while((c = getchar()) != '\n'){
        response[i] = c;
        ++i;
    }

    response[i] = '\0';
	info_ptr->response = malloc(sizeof(char) * 128);
    strcpy(info_ptr->response, response);
    printf("received: %s\n", info_ptr->response);
	/*	Command received, send it to get interpretted	*/
	if (strcmp(info_ptr->programMode, "fk") == 0) {
		interpret_command_fk(info_ptr, move_ptr);
		free(info_ptr->response); // not free'd yet
	}
	else if (strcmp(info_ptr->programMode, "ik") == 0) {
		interpret_command_ik(info_ptr, move_ptr, true); // response is free'd in interpret_command
	}
}


void interpret_command_ik(info* info_ptr, move* move_ptr, bool commandLine) {

	float duty = 1.0;
	/*	Mode is for IK, move target	object	*/
	if (!commandLine) {
		/*	Recieved command from joystick, need to determine % to move by	*/
		char buffer[100];
		strcpy(buffer, info_ptr->response);

		char* token = strtok(buffer, " ");
		token = strtok(NULL, " ");
		char* err;
		int joystickValue = strtol(token, &err, 10);
		duty = (float)(joystickValue / 32767.0);		//32767 is toggled all the way

	}

	printf("	Duty:	%f\n", duty);
	
	if (info_ptr->response[0] == 'w') { move_target_vrep(info_ptr, move_ptr, 'w', duty); }
	if (info_ptr->response[0] == 'a') { move_target_vrep(info_ptr, move_ptr, 'a', duty); }
	if (info_ptr->response[0] == 's') { move_target_vrep(info_ptr, move_ptr, 's', duty); }
	if (info_ptr->response[0] == 'd') { move_target_vrep(info_ptr, move_ptr, 'd', duty); }
	if (info_ptr->response[0] == '-') { move_target_vrep(info_ptr, move_ptr, '-', duty); }
	if (info_ptr->response[0] == '+') { move_target_vrep(info_ptr, move_ptr, '+', duty); }




	free(info_ptr->response);
}


void interpret_command_fk(info* info_ptr, move* move_ptr) {
	printf("interpret_command\n");


	/*	Forward Kinematics Mode Interpretting	*/
	char joint[6] = { '1', '2', '3', '4', '5', '6' };
	int check = 1;
	for (int j = 0; j < 6; j++) {
		if (info_ptr->response[0] == joint[j]) {
			check = 0;
		}
	}


	if ((strlen(info_ptr->response) > 3) && (info_ptr->response[1] == ' ')) { 
		/*	Valid command, proceed	*/	
		check = 0; 
	}
	if ((strlen(info_ptr->response) < 4) && (info_ptr->response[0] == 'f') && info_ptr->response[1] == 'k') { 
		/*	was asked to get the joint angles using the classic DH parameters and FK	*/
		fk_classic(move_ptr, info_ptr); 
	}

	if (check) { 
		/*	Invalid command, prompt for a new one	*/	
		printf("*%s\n", info_ptr->response); 
		free(info_ptr->response);
		get_command(info_ptr, move_ptr); 
	}

	/*	Interpret the fk command, get the joint # and angle to move	*/
	char response[128];
	char* token;
	strcpy(response, info_ptr->response);
	token = strtok(response, " ");
	token = strtok(NULL, " ");
	
	double ang;
	char* ptr;
	ang = strtol(token, &ptr, 10);
	if ((ang < -360) || (ang > 360)) {
		printf("invalid angle %f\n", ang);
		free(info_ptr->response);
		get_command(info_ptr, move_ptr);
	}
	int jointNum = 0;
	if (info_ptr->response[0] == joint[0]) {
		jointNum = 1;
	}
	else if (info_ptr->response[0] == joint[1]) {
		jointNum = 2;
	}
	else if (info_ptr->response[0] == joint[2]) {
		jointNum = 3;
	}
	else if (info_ptr->response[0] == joint[3]) {
		jointNum = 4;
	}
	else if (info_ptr->response[0] == joint[4]) {
		jointNum = 5;
	}
	else if (info_ptr->response[0] == joint[4]) {
		jointNum = 6;
	}

	/*	Move the joint a specified angle	*/
	move_joint_angle_vrep(info_ptr, move_ptr, jointNum, ang);

	/*	end of interpreting, free the response	*/	
	free(info_ptr->response);
}


void get_object_names_vrep(info* info_ptr){
    /*  Retrieves object names for corresponding object handle,
     *  stored in info struct
     *
     *  Calls a custom function in VRep main script, which takes in an
     *  object handle and returns the object's name.
    */
    int replySize[1] = {1};
    

    for(int i = 0; i < info_ptr->objectCount; ++i){
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
		else {
			printf("%d :	%s\n", i, replyData);
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
			if (strcmp(token, "Target") == 0) {
				info_ptr->targetHandle = i+1;
			}
            token = strtok(NULL, underScore);
            if (count == 5){
                info_ptr->isJoint[i] = 1;
            }
        }
    }
    printf("Got object names\n");
}



void initial_arm_config_vrep(info* info_ptr, move* move_ptr) {
	/* puts the arm into a starting pos, straight up, by rotating joints 4 and 5
	*  the joint handles are also found, and angles of each using get_joint_angles */
	printf("initial_arm_config_vrep\n");
	int i = 0;
	int num = 0;
	info_ptr->jacoArmJointHandles = malloc(sizeof(int) * 6);
	printf("jacoarmjointhandles malloced %d\n", info_ptr->objectCount);

	while (num < 6) {

		if (info_ptr->isJoint[i]) {
			//printf("is joint: %s\n", info_ptr->objectNames[i]);
			info_ptr->jacoArmJointHandles[num] = info_ptr->objectHandles[i];
			printf("joint handle: %d\n", info_ptr->jacoArmJointHandles[num]);
			++num;
		}
		++i;
		if (i > info_ptr->objectCount) {
			printf("i: %d\n", i);
			exit(1);
		}
	}

	get_joint_angles_vrep(info_ptr, move_ptr);
	
	int ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[5-1], 5, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[5-1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[5 - 1], 0, simx_opmode_oneshot_wait);
	
	ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 5, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 0, simx_opmode_oneshot_wait);

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

	char line2[256]; 
	sprintf(line2, "%d Target\n", info_ptr->targetHandle);
	fputs(line2, object_fp);
    fflush(object_fp);
    fclose(object_fp);

}


void read_object_info(info* info_ptr, char* filename) {
	/*	read's in the file that contains the object names
	*	and handles. Updates the info_ptr struct with information	*/

	FILE* objectFilePtr = fopen(filename, "r");
	if (objectFilePtr == NULL) {
		fprintf(stdout, "Failed to open file %s for reading\n", filename);
		exit(1);
	}

	char buffer[BUFSIZE];
	int fileLength = 0;
	char c = fgetc(objectFilePtr);

	while ((c != EOF) && (fileLength < BUFSIZE)) {
		/*	Load as much of the file into memory as possible,
		*	save the length of the file in characters so the end
		*	position is known	*/
		buffer[fileLength] = c;
		c = fgetc(objectFilePtr);
		++fileLength;
	}

	c = '\0';
	//int count;
	int i = 0;
	int stable = 1;

	while (i < fileLength) {
		/*	go through every character in the file's loaded buffer,
		*	breaking words at spaces and new lines	*/
		c = buffer[i];
		++i;
		if (c == EOF) {
			stable = 0;
		}
		int spaceToggle = 0;
		int wordHandleIt = 0;
		int objectNameIt = 0;
		char wordHandle[6];
		int objectHandle = 0;
		char objectName[35];
		while (c != '\n') {
			if (!spaceToggle) {
				wordHandle[wordHandleIt] = c;
				++wordHandleIt;
			}
			else {
				if (c != ' ') {
					objectName[objectNameIt] = c;
					++objectNameIt;
				}
			}

			c = buffer[i];
			++i;
			if (c == ' '){
				/*	The object handle of a joint has been stored in a temporary array
				*	and needs to be saved	*/
				spaceToggle = true;

				wordHandle[wordHandleIt] = '\0';
				char* err;
				objectHandle = strtol(wordHandle, &err, 10);
				info_ptr->isJoint[objectHandle] = 1;
			} else if (c == '\n') {
				/*	The name corresponding to the last saved objectHandle has been stored
				*	and needs to be perminantly saved, as this will allow finding the
				*	targeHandle etc later.	*/
				objectName[objectNameIt] = '\0';
				info_ptr->objectNames[objectHandle] = malloc(sizeof(char)* (strlen(objectName)));
				strcpy(info_ptr->objectNames[objectHandle], objectName);
			}
		}
	}

	for (int ob = 0; ob < info_ptr->objectCount; ob++) {
		/*	Fill in any information that was missed, it is not necessary to know the 
		*	names of objects that aren't joints, however for analysis later, the
		*	isJoint array must be full of 0's or 1's so this needs to be filled in.
		*	The Target must also be found and recorded as targetHandle	*/
		if (info_ptr->isJoint[ob] != 1) {
			info_ptr->isJoint[ob] = 0;
			info_ptr->objectNames[ob] = malloc(sizeof(char)*3);
			info_ptr->objectNames[ob] = '\0';
		}
		else if (strcmp(info_ptr->objectNames[ob], "Target") == 0) {
			info_ptr->isJoint[ob] = 0;
			info_ptr->targetHandle = ob;
		}
	}
	printf("This is the target's object handle: %d\n", info_ptr->targetHandle);
}


void get_orientation_vrep(info* info_ptr, simxFloat* orientation, int handle, int relativeHandle) {
	/*	Updates the position vector with relativeHandle's orientation relative to the base of the arm	*/
	int jointHandle1 = relativeHandle;
	if (relativeHandle == -1) {
		for (int i = 0; i < info_ptr->objectCount; i++) {
			if (info_ptr->isJoint[i]) {
				jointHandle1 = i;
			}
		}
	}
	// get position of handle relative to jaco_joint_1
	simxGetObjectOrientation(info_ptr->clientID, handle, jointHandle1, &orientation, simx_opmode_blocking);
}


void get_position_vrep(info* info_ptr, simxFloat* position, int handle) {
	/*	Updates the position vector with relativeHandle's position relative to the base of the arm	*/
	 
	int jointHandle1 = 0;
	for (int i = 0; i < info_ptr->objectCount; i++) {
		if (info_ptr->isJoint[i]) {
			jointHandle1 = i;
		}
	}

	// get position of handle relative to jaco_joint_1
	simxGetObjectPosition(info_ptr->clientID, handle, jointHandle1, &position, simx_opmode_blocking);
}


void get_joint_angles_vrep(info* info_ptr, move* move_ptr) {
	/* Updates info struct with all jaco arm joint current angles
	*  joints 4 and 5 are initially adjusted to take on FK values,
	*  the other joints are offset by 180 degrees */
	printf("get_joint_angles_vrep\n");
	int count = 0;
	while (count < 6) {
		printf(".%d\n", info_ptr->jacoArmJointHandles[count]);
		simxFloat position;
		int ret = simxGetJointPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[count], &position, simx_opmode_blocking);
		move_ptr->currAng[count] = 0;
		if ((count == (5 - 1)) || (count == (4 - 1))) {
			move_ptr->currAng[count] = (double)position;
		}
		else {
			move_ptr->currAng[count] = (double)position - 3.141592;
			if (move_ptr->currAng[count] < 0) {
				move_ptr->currAng[count] += 2 * 3.141592;
			}
		}

		printf("this is angle: %d, %f, %f\n", info_ptr->jacoArmJointHandles[count], position, move_ptr->currAng[count]);
		//printf("this is angle: %s, %f\n", info_ptr->objectNames[info_ptr->jacoArmJointHandles[count]], position);
		++count;
	}
}


void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang) {
	/* takes in a joint number between 1 and 6, these are translated into
	*  object handles, and the arm is moved via external command */

	get_joint_angles_vrep(info_ptr, move_ptr);

	double jointAngle = move_ptr->currAng[jointNum - 1] + (ang*3.141592 / 180.0);
	if ((jointNum != 4) && (jointNum != 5)) {
		jointAngle += 3.141592;
	}
	jointAngle = fmod(jointAngle, 2 * 3.141592);
	printf("moving joint %d : %f : %f\n", jointNum, jointAngle, ang);
	int ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 20, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], jointAngle, simx_opmode_oneshot_wait);

	get_command(info_ptr, move_ptr);
}


void move_target_vrep(info * info_ptr, move * move_ptr, char direction, float duty)
{
	/*	Get the target's position relative to the base of the arm	*/
	printf("move_target(), %d\n", info_ptr->targetHandle); fflush(stdout);
	//simxFloat* position = malloc(sizeof(int) * 3);
	//simxFloat position = malloc(sizeof(simxFloat)*3);
	simxFloat position[3];
	simxFloat orientation[3];
	simxGetObjectPosition(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &position, simx_opmode_blocking);
	simxGetObjectOrientation(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &orientation, simx_opmode_blocking);
	printf("%f %f %f	%f %f %f\n", position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]); fflush(stdout);
	if (direction == 'w') { position[1] += (simxFloat)(0.01 * duty); }
	if (direction == 's') { position[1] -= (simxFloat)(0.01 * duty); }
	if (direction == 'a') { position[0] += (simxFloat)(0.01 * duty); }
	if (direction == 'd') { position[0] -= (simxFloat)(0.01 * duty); }
	if (direction == '+') { position[2] += (simxFloat)(0.01 * duty); }
	if (direction == '-') { position[2] -= (simxFloat)(0.01 * duty); }
	printf("%f %f %f\n", position[0], position[1], position[2]); fflush(stdout);

	simxSetObjectPosition(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &position, simx_opmode_oneshot);

}