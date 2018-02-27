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

#define BUFSIZE 4096 

HANDLE  hConsoleOut;                 // Handle to the console   
HANDLE  hRunMutex;                   // "Keep Running" mutex   
HANDLE  hScreenMutex;                // "Screen update" mutex  
int     ThreadNr;                    // Number of threads started   
CONSOLE_SCREEN_BUFFER_INFO csbiInfo; // Console information 



typedef struct info {
	
    int clientID;						/*	ID for networking	*/
    int objectCount;					/*	Number of objects in the scene	*/	
    int* objectHandles;					/*	The ID corresponding to the i'th object	*/
    int* isJoint;						/*	Boolean array, for i'th handle as a joint	*/
    char** objectNames;					/*	Array of names of objects in scene	*/
	int* jacoArmJointHandles;			/*	joint ID array	*/
    char* response;

	int targetHandle;

	char programMode[3];


} info;



info* makeInfo(void); 
move* makeMove(void);
void get_object_names_vrep(info* info_ptr);
void write_object_info(info* info_ptr, char* filename);
void get_command(info* info_ptr, move* move_ptr);
void get_joint_angles_vrep(info* info_ptr, move* move_ptr);
void initial_arm_config_vrep(info* info_ptr, move* move_ptr);
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void interpret_command(info* info_ptr, move* move_ptr);
void CreateChildProcess(void);
void WriteToPipe(void);
void ReadFromPipe(void);
void get_position(info* info_ptr, simxFloat* startPosition, int relativeHandle);
void move_target(info* info_ptr, move* move_ptr, char direction);
void fk_classic(move* move_ptr, info* info_ptr);
void read_object_info(info* info_ptr, char* filename);

void initialise_program(info* info_ptr, char argc, char** argv) {
	/*	Determine the mode of the Jaco arm in Vrep from input commands	*/
	char inputBuffer[10];
	if (argc == 1) {
		printf("Usage: vrepClientProgram.exe fk/ik [object filename] [IP Address] [port]");
		printf("\nPlease specify a Kinematics mode (ik/fk)>> "); fflush(stdout);
		char c = getchar();
		int i = 0;
		while ((c != '\n') && (i < 10)) {
			/*	store input	*/
			inputBuffer[i] = c;
			c = getchar();
			++i;
		}
	}
	else {
		strcpy(inputBuffer, argv[2]);
	}
	/*	Search for mode in input buffer	*/
	for (int j = 0; j < 9; j++) {
		if (((inputBuffer[j] == 'i') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'I') && (inputBuffer[j + 1] == 'K'))) {
			strcpy(info_ptr->programMode, "ik");
		}
		else if (((inputBuffer[j] == 'f') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'F') && (inputBuffer[j + 1] == 'K'))) {
			strcpy(info_ptr->programMode, "fk");
		}
	}
	printf("%s program mode was set\n", info_ptr->programMode);

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
		/*  calls custom function in VRep Main */
		get_object_names_vrep(info_ptr);
	}
	else if (argc >= 3) {
		/*	Check if need to fill in the file or if it is ready for reading	*/
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
		return 1;
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

 //   /*  write information to a file */
	//if (en_name) {
	//	if (argc == 3) {
	//		/* filename specified */
	//		write_object_info(info_ptr, argv[2]);
	//	}
	//	else if (argc == 2) {
	//		write_object_info(info_ptr, "objects");
	//	}
	//}
 //   printf("Written to file\n");

	

	/*  Print a sample name to check */
	//printf("aa: %s\n", info_ptr->objectNames[40]);


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

	while (1) {

		/*	Need to work out how to choose between command or joystick based control	*/
		if (joystick_input_available) {
			/*	A joystick command is available to action on	*/
			int* arr;
			arr = joystick_get_char();
			printf("arr:	%d %d \n", arr[0], arr[1]);
		}

		/*	Instead of getting input from joystick, get from command line input	*/
		get_command(info_ptr, move_ptr);
			
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
	interpret_command(info_ptr, move_ptr);
	free(info_ptr->response);
}


void interpret_command(info* info_ptr, move* move_ptr) {
	printf("interpret_command\n");
	char joint[6] = { '1', '2', '3', '4', '5', '6' };
	int check = 1;
	for (int j = 0; j < 6; j++) {
		if (info_ptr->response[0] == joint[j]) {
			check = 0;
		}
	}
	if ((strlen(info_ptr->response) > 3) && (info_ptr->response[1] == ' ')) { check = 0; }
	if ((strlen(info_ptr->response) < 3) && (info_ptr->response[0] == 'w')) { move_target(info_ptr, move_ptr, 'w'); }
	if ((strlen(info_ptr->response) < 3) && (info_ptr->response[0] == 'a')) { move_target(info_ptr, move_ptr, 'a'); }
	if ((strlen(info_ptr->response) < 3) && (info_ptr->response[0] == 's')) { move_target(info_ptr, move_ptr, 's'); }
	if ((strlen(info_ptr->response) < 3) && (info_ptr->response[0] == 'd')) { move_target(info_ptr, move_ptr, 'd'); }
	if ((strlen(info_ptr->response) < 4) && (info_ptr->response[0] == 'i') && info_ptr->response[1] == 'k') { fk_classic(move_ptr, info_ptr); }

	if (check) { 
		printf("*%s\n", info_ptr->response); 
		free(info_ptr->response);
		get_command(info_ptr, move_ptr); 
	}
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

	move_joint_angle_vrep(info_ptr, move_ptr, jointNum, ang);


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
			if (strcmp(token, "Target") > 0) {
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

void get_joint_angles_vrep(info* info_ptr, move* move_ptr) {
	/* Updates info struct with all jaco arm joint current angles 
	*  joints 4 and 5 are initially adjusted to take on FK values,
	*  the other joints are offset by 180 degrees */
	printf("get_joint_angles_vrep\n");
	int count = 0;
	while (count < 6){
			printf(".%d\n", info_ptr->jacoArmJointHandles[count]);
			simxFloat position;
			int ret = simxGetJointPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[count], &position, simx_opmode_blocking);
			move_ptr->currAng[count] = 0;
			if ((count == (5 - 1)) || (count == (4-1))) {
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

	double jointAngle = move_ptr->currAng[jointNum - 1] + (ang*3.141592/180.0);
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
		buffer[fileLength] = c;
		c = fgetc(objectFilePtr);
		++fileLength;
	}

	c = '\0';
	int count;
	int i = 0;
	//for (i = 0; i < fileLength; i++) {
	int stable = 1;

	while (i < fileLength) {

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
				spaceToggle = true;

				wordHandle[wordHandleIt] = '\0';
				char* err;
				objectHandle = strtol(wordHandle, &err, 10);
				info_ptr->isJoint[objectHandle] = 1;
				//printf("is joint %d\n", objectHandle);
			} else if (c == '\n') {
				objectName[objectNameIt] = '\0';
				info_ptr->objectNames[objectHandle] = malloc(sizeof(char)* (strlen(objectName)));
				strcpy(info_ptr->objectNames[objectHandle], objectName);
				//printf("is name ^%s^\n", info_ptr->objectNames[objectHandle]);
			}
		}
	}
}



void define_classic_parameters(move* move_ptr) {
	/*	Stores in memory the forward kinematic
	*	parameters of the Jaco arm as per the classic DH solution
	*/

	/* Define joint lengths */
	move_ptr->lengthD = malloc(sizeof(double) * 7);
	move_ptr->lengthD[0] = 9.8;			// hand offset				e2
	move_ptr->lengthD[1] = 275.5;		// length of joint 1		D1
	move_ptr->lengthD[2] = 410.0;		// length of joint 2		D2
	move_ptr->lengthD[3] = 207.3;		//							D3
	move_ptr->lengthD[4] = 74.3;		//							D4
	move_ptr->lengthD[5] = 74.3;		//							D5	
	move_ptr->lengthD[6] = 168.7;		//							D6

	/* Define DH Parameters */
	move_ptr->alpha = malloc(sizeof(double) * 7);
	move_ptr->alpha[0] = 0.0;				// redundant, will start at 1 for joint 1
	move_ptr->alpha[1] = 3.141592 / 2.0;
	move_ptr->alpha[2] = 3.141592;
	move_ptr->alpha[3] = 3.141592 / 2.0;
	move_ptr->alpha[4] = 55.0*3.141592 / 180.0;
	move_ptr->alpha[5] = 55.0*3.141592 / 180.0;
	move_ptr->alpha[6] = 3.141592;

	move_ptr->a_i = malloc(sizeof(double) * 7);
	for (int x = 0; x < 7; x++) {
		
		if (x == 2) {
			move_ptr->alpha[x] = move_ptr->lengthD[2];
		}
		else {
			move_ptr->alpha[x] = 0.0;
		}
	}

	double aa = sin(55.0*3.141592 / 360.0) / sin(55.0*3.141592 / 180.0);

	move_ptr->d_i = malloc(sizeof(double) * 7);
	move_ptr->d_i[0] = 0.0;
	move_ptr->d_i[1] = move_ptr->lengthD[1];
	move_ptr->d_i[2] = 0.0;
	move_ptr->d_i[3] = move_ptr->lengthD[0];
	move_ptr->d_i[4] = -(move_ptr->lengthD[3] + move_ptr->lengthD[4]*aa);
	move_ptr->d_i[5] = -(move_ptr->lengthD[4]*aa + move_ptr->lengthD[5] * aa);
	move_ptr->d_i[6] = -(move_ptr->lengthD[6] + move_ptr->lengthD[5] * aa);

}



void fk_classic(move* move_ptr, info* info_ptr) {
	/*	Calculates the position of the tip using classic DH parameters
	*	ret = T * pt	
	*/	
	
	/* Get current angles*/
	double q1, q2, q3, q4, q5, q6;
	q1 = move_ptr->currAng[0];
	q2 = move_ptr->currAng[1];
	q3 = move_ptr->currAng[2];
	q4 = move_ptr->currAng[3];
	q5 = move_ptr->currAng[4];
	q6 = move_ptr->currAng[5];
	printf("current angles: %f %f %f %f %f %f\n", q1, q2, q3, q4, q5, q6);
	/* Define a reference position	*/
	double pt[4][1];
	pt[0][0] = 0.0;
	pt[1][0] = 98;
	pt[2][0] = 1126.9;
	pt[3][0] = 0.0;

	/* Compute the transform matrix given angles*/
	double T[4][4];

	T[1 - 1][1 - 1] = 0.75*cos(q4)*sin(q1)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q2)*sin(q3)*sin(q6) - 0.43301270189221932338186158537647*cos(q1)*cos(q3)*sin(q2)*sin(q6) - 0.25*cos(q4)*cos(q5)*sin(q1)*sin(q6) - 0.5*cos(q4)*cos(q6)*sin(q1)*sin(q5) - 1.0*cos(q5)*cos(q6)*sin(q1)*sin(q4) + 0.5*sin(q1)*sin(q4)*sin(q5)*sin(q6) + 0.75*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q6) + 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5) - 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q6) - 0.86602540378443864676372317075294*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q5) + 0.75*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 0.5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - 0.25*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) - 0.5*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) + cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - 0.5*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 0.25*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 0.5*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
	T[1 - 1][2 - 1] = 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q6)*sin(q2) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q6)*sin(q3) - 0.75*cos(q4)*cos(q6)*sin(q1) + 0.25*cos(q4)*cos(q5)*cos(q6)*sin(q1) - 0.5*cos(q4)*sin(q1)*sin(q5)*sin(q6) - 1.0*cos(q5)*sin(q1)*sin(q4)*sin(q6) - 0.5*cos(q6)*sin(q1)*sin(q4)*sin(q5) - 0.75*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3) + 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q2) - 0.75*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3)*sin(q5)*sin(q6) - 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)*sin(q5)*sin(q6) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 0.5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) + 0.25*cos(q1)*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - 0.5*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + 0.5*cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + 0.25*cos(q1)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 0.5*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
	T[1 - 1][3 - 1] = 0.86602540378443864676372317075294*sin(q1)*sin(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q4)*sin(q1) - 0.25*cos(q1)*cos(q2)*sin(q3) + 0.25*cos(q1)*cos(q3)*sin(q2) - 0.43301270189221932338186158537647*cos(q4)*cos(q5)*sin(q1) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*sin(q4) + 0.75*cos(q1)*cos(q2)*cos(q5)*sin(q3) - 0.75*cos(q1)*cos(q3)*cos(q5)*sin(q2) - 0.43301270189221932338186158537647*cos(q1)*sin(q2)*sin(q3)*sin(q4) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 0.86602540378443864676372317075294*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 0.43301270189221932338186158537647*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4);
	T[1 - 1][4 - 1] = 9.8000000000000211859935286588393*sin(q1) + 410.0*cos(q1)*cos(q2) - 165.92424280921737560930281782652*cos(q4)*sin(q1) + 183.2484856184347722889290885445*sin(q1)*sin(q4)*sin(q5) - 345.99353125177566425918485037982*cos(q1)*cos(q2)*sin(q3) + 345.99353125177566425918485037982*cos(q1)*cos(q3)*sin(q2) - 91.62424280921738614446454427225*cos(q4)*cos(q5)*sin(q1) - 165.92424280921737560930281782652*cos(q1)*cos(q2)*cos(q3)*sin(q4) + 158.697843750591871980759606231*cos(q1)*cos(q2)*cos(q5)*sin(q3) - 158.697843750591871980759606231*cos(q1)*cos(q3)*cos(q5)*sin(q2) - 165.92424280921737560930281782652*cos(q1)*sin(q2)*sin(q3)*sin(q4) - 183.2484856184347722889290885445*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 91.62424280921738614446454427225*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 183.2484856184347722889290885445*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 91.62424280921738614446454427225*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4);
	T[2 - 1][1 - 1] = 0.25*cos(q1)*cos(q4)*cos(q5)*sin(q6) - 0.75*cos(q1)*cos(q4)*sin(q6) + 0.5*cos(q1)*cos(q4)*cos(q6)*sin(q5) + cos(q1)*cos(q5)*cos(q6)*sin(q4) + 0.43301270189221932338186158537647*cos(q2)*sin(q1)*sin(q3)*sin(q6) - 0.43301270189221932338186158537647*cos(q3)*sin(q1)*sin(q2)*sin(q6) - 0.5*cos(q1)*sin(q4)*sin(q5)*sin(q6) + 0.75*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) + 0.43301270189221932338186158537647*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q6) + 0.86602540378443864676372317075294*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q5) - 0.43301270189221932338186158537647*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q6) - 0.86602540378443864676372317075294*cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q5) + 0.75*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1) - 0.5*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q6) - 0.25*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*sin(q6) - 0.5*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4)*sin(q5) + cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3) - 0.5*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 0.25*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 0.5*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
	T[2 - 1][2 - 1] = 0.75*cos(q1)*cos(q4)*cos(q6) - 0.25*cos(q1)*cos(q4)*cos(q5)*cos(q6) - 0.43301270189221932338186158537647*cos(q2)*cos(q6)*sin(q1)*sin(q3) + 0.43301270189221932338186158537647*cos(q3)*cos(q6)*sin(q1)*sin(q2) + 0.5*cos(q1)*cos(q4)*sin(q5)*sin(q6) + cos(q1)*cos(q5)*sin(q4)*sin(q6) + 0.5*cos(q1)*cos(q6)*sin(q4)*sin(q5) - 0.75*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) - 0.43301270189221932338186158537647*cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3) + 0.43301270189221932338186158537647*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2) - 0.75*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) + 0.5*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) + 0.25*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4) - 0.5*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)*sin(q6) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6) + 0.5*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + 0.25*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 0.5*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
	T[2 - 1][3 - 1] = 0.43301270189221932338186158537647*cos(q1)*cos(q4) + 0.43301270189221932338186158537647*cos(q1)*cos(q4)*cos(q5) - 0.25*cos(q2)*sin(q1)*sin(q3) + 0.25*cos(q3)*sin(q1)*sin(q2) - 0.86602540378443864676372317075294*cos(q1)*sin(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*sin(q1)*sin(q4) + 0.75*cos(q2)*cos(q5)*sin(q1)*sin(q3) - 0.75*cos(q3)*cos(q5)*sin(q1)*sin(q2) - 0.43301270189221932338186158537647*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 0.86602540378443864676372317075294*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4) - 0.86602540378443864676372317075294*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5) - 0.43301270189221932338186158537647*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4);
	T[2 - 1][4 - 1] = 165.92424280921737560930281782652*cos(q1)*cos(q4) - 9.8000000000000211859935286588393*cos(q1) + 410.0*cos(q2)*sin(q1) + 91.62424280921738614446454427225*cos(q1)*cos(q4)*cos(q5) - 345.99353125177566425918485037982*cos(q2)*sin(q1)*sin(q3) + 345.99353125177566425918485037982*cos(q3)*sin(q1)*sin(q2) - 183.2484856184347722889290885445*cos(q1)*sin(q4)*sin(q5) - 165.92424280921737560930281782652*cos(q2)*cos(q3)*sin(q1)*sin(q4) + 158.697843750591871980759606231*cos(q2)*cos(q5)*sin(q1)*sin(q3) - 158.697843750591871980759606231*cos(q3)*cos(q5)*sin(q1)*sin(q2) - 165.92424280921737560930281782652*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 183.2484856184347722889290885445*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - 91.62424280921738614446454427225*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4) - 183.2484856184347722889290885445*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5) - 91.62424280921738614446454427225*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4);
	T[3 - 1][1 - 1] = 0.43301270189221932338186158537647*sin(q2)*sin(q3)*sin(q6) + 0.43301270189221932338186158537647*cos(q2)*cos(q3)*sin(q6) + 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q5)*sin(q6) + 0.86602540378443864676372317075294*cos(q2)*cos(q3)*cos(q6)*sin(q5) - 0.75*cos(q2)*sin(q3)*sin(q4)*sin(q6) + 0.75*cos(q3)*sin(q2)*sin(q4)*sin(q6) + 0.43301270189221932338186158537647*cos(q5)*sin(q2)*sin(q3)*sin(q6) + 0.86602540378443864676372317075294*cos(q6)*sin(q2)*sin(q3)*sin(q5) - 1.0*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 0.5*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + 0.25*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + 0.5*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - 0.5*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) - 0.25*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - 0.5*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5);
	T[3 - 1][2 - 1] = 0.75*cos(q2)*cos(q6)*sin(q3)*sin(q4) - 0.43301270189221932338186158537647*cos(q6)*sin(q2)*sin(q3) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q5)*cos(q6) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q6) - 0.75*cos(q3)*cos(q6)*sin(q2)*sin(q4) + 0.86602540378443864676372317075294*cos(q2)*cos(q3)*sin(q5)*sin(q6) - 0.43301270189221932338186158537647*cos(q5)*cos(q6)*sin(q2)*sin(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 1.0*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) - 0.5*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - 0.25*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + 0.5*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + 0.25*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + 0.5*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - 0.5*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6);
	T[3 - 1][3 - 1] = 0.75*cos(q2)*cos(q3)*cos(q5) - 0.25*sin(q2)*sin(q3) - 0.25*cos(q2)*cos(q3) + 0.43301270189221932338186158537647*cos(q2)*sin(q3)*sin(q4) - 0.43301270189221932338186158537647*cos(q3)*sin(q2)*sin(q4) + 0.75*cos(q5)*sin(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q2)*cos(q4)*sin(q3)*sin(q5) + 0.43301270189221932338186158537647*cos(q2)*cos(q5)*sin(q3)*sin(q4) - 0.86602540378443864676372317075294*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 0.43301270189221932338186158537647*cos(q3)*cos(q5)*sin(q2)*sin(q4);
	T[3 - 1][4 - 1] = 410.0*sin(q2) - 345.99353125177566425918485037982*cos(q2)*cos(q3) - 345.99353125177566425918485037982*sin(q2)*sin(q3) + 158.697843750591871980759606231*cos(q2)*cos(q3)*cos(q5) + 165.92424280921737560930281782652*cos(q2)*sin(q3)*sin(q4) - 165.92424280921737560930281782652*cos(q3)*sin(q2)*sin(q4) + 158.697843750591871980759606231*cos(q5)*sin(q2)*sin(q3) + 183.2484856184347722889290885445*cos(q2)*cos(q4)*sin(q3)*sin(q5) + 91.62424280921738614446454427225*cos(q2)*cos(q5)*sin(q3)*sin(q4) - 183.2484856184347722889290885445*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 91.62424280921738614446454427225*cos(q3)*cos(q5)*sin(q2)*sin(q4) + 275.5000000000000006000769315822;
	T[4 - 1][1 - 1] = 0.0;
	T[4 - 1][2 - 1] = 0.0;
	T[4 - 1][3 - 1] = 0.0;
	T[4 - 1][4 - 1] = 1.0;

	/* define the current position*/
	double ret[4][1];
	ret[0][0] = 0.0;
	ret[1][0] = 0.0;
	ret[2][0] = 0.0;
	ret[3][0] = 0.0;

	/* Compute the position*/
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 4; k++) {
				ret[i][j] += T[i][k] * pt[k][j];
			}

		}
	}
	printf("Forward: %f %f %f\n", ret[0][1], ret[1][0], ret[2][0]);
	simxFloat position[3];
	simxGetObjectPosition(info_ptr->clientID, 33, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	simxGetObjectPosition(info_ptr->clientID, 30, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	simxGetObjectPosition(info_ptr->clientID, 27, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	simxGetObjectPosition(info_ptr->clientID, 24, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	simxGetObjectPosition(info_ptr->clientID, 21, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	simxGetObjectPosition(info_ptr->clientID, 18, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);

}


void fk_mod(move* move_ptr) {
	/* Updates current position vector signing modified DH parameters
	*  and forward kinematics */


	/* Get current angles*/
	double q1, q2, q3, q4, q5, q6;
	q1 = move_ptr->currAng[0];
	q2 = move_ptr->currAng[1];
	q3 = move_ptr->currAng[2];
	q4 = move_ptr->currAng[3];
	q5 = move_ptr->currAng[4];
	q6 = move_ptr->currAng[5];
	/* Define a reference position	*/
	double pt[4][1];
	pt[0][0] = move_ptr->currPos[0];
	pt[1][0] = move_ptr->currPos[1];
	pt[2][0] = move_ptr->currPos[3];
	pt[3][0] = 0.0;

	/* Compute the transform matrix given angles*/
	double T[4][4];

	T[1 - 1][1 - 1] = 0.5*cos(q1)*cos(q5)*sin(q2)*sin(q6) + cos(q1)*cos(q6)*sin(q2)*sin(q5) + 0.5*cos(q2)*cos(q5)*sin(q1)*sin(q6) + cos(q2)*cos(q6)*sin(q1)*sin(q5) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q6) + 0.86602540378443864676372317075294*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q6) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 0.5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - 1.0*cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) - 1.0*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2) + 0.5*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + 0.5*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5)*sin(q6) + cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 0.5*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
	T[1 - 1][2 - 1] = 0.25*cos(q1)*cos(q5)*cos(q6)*sin(q2) - 0.75*cos(q2)*cos(q5)*sin(q1) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*sin(q4) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q4)*sin(q3) - 0.75*cos(q1)*cos(q5)*sin(q2) + 0.25*cos(q2)*cos(q5)*cos(q6)*sin(q1) + 0.43301270189221932338186158537647*cos(q3)*sin(q1)*sin(q2)*sin(q4) + 0.43301270189221932338186158537647*cos(q4)*sin(q1)*sin(q2)*sin(q3) - 0.5*cos(q1)*sin(q2)*sin(q5)*sin(q6) - 0.5*cos(q2)*sin(q1)*sin(q5)*sin(q6) + 0.75*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - 0.75*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.75*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) + 0.43301270189221932338186158537647*cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q4) + 0.43301270189221932338186158537647*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3) + 0.75*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - 0.25*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) + 0.5*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + 0.25*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + 0.5*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q6) + 0.25*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q5) - 0.5*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 0.25*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
	T[1 - 1][3 - 1] = 0.25*cos(q3)*sin(q1)*sin(q2)*sin(q4) - 0.43301270189221932338186158537647*cos(q2)*cos(q5)*sin(q1) - 0.25*cos(q1)*cos(q2)*cos(q3)*sin(q4) - 0.25*cos(q1)*cos(q2)*cos(q4)*sin(q3) - 0.43301270189221932338186158537647*cos(q1)*cos(q5)*cos(q6)*sin(q2) - 0.43301270189221932338186158537647*cos(q2)*cos(q5)*cos(q6)*sin(q1) - 0.43301270189221932338186158537647*cos(q1)*cos(q5)*sin(q2) + 0.25*cos(q4)*sin(q1)*sin(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*sin(q2)*sin(q5)*sin(q6) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q5)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 0.75*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 0.75*cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) - 0.75*cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q4) - 0.75*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3) + 0.43301270189221932338186158537647*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - 0.86602540378443864676372317075294*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q6) - 0.43301270189221932338186158537647*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q5) + 0.86602540378443864676372317075294*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 0.43301270189221932338186158537647*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
	T[1 - 1][4 - 1] = 410.0*cos(q1)*cos(q2)*cos(q3) - 259.99712500078919711005658114022*cos(q2)*sin(q1) - 259.99712500078919711005658114022*cos(q1)*sin(q2) - 183.2484856184347722889290885445*cos(q1)*cos(q5)*sin(q2) - 183.2484856184347722889290885445*cos(q2)*cos(q5)*sin(q1) - 410.0*cos(q3)*sin(q1)*sin(q2) - 191.59281250197295776160899549723*cos(q1)*cos(q2)*cos(q3)*sin(q4) - 191.59281250197295776160899549723*cos(q1)*cos(q2)*cos(q4)*sin(q3) + 191.59281250197295776160899549723*cos(q3)*sin(q1)*sin(q2)*sin(q4) + 191.59281250197295776160899549723*cos(q4)*sin(q1)*sin(q2)*sin(q3) + 183.2484856184347722889290885445*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 183.2484856184347722889290885445*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 183.2484856184347722889290885445*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) + 183.2484856184347722889290885445*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
	T[2 - 1][1 - 1] = 0.5*cos(q5)*sin(q1)*sin(q2)*sin(q6) - 1.0*cos(q1)*cos(q2)*cos(q6)*sin(q5) - 0.5*cos(q1)*cos(q2)*cos(q5)*sin(q6) + cos(q6)*sin(q1)*sin(q2)*sin(q5) - 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - 0.86602540378443864676372317075294*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - 0.86602540378443864676372317075294*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) - 0.86602540378443864676372317075294*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q6) + cos(q1)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1) - 0.5*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) - 1.0*cos(q1)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 0.5*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q6) - 1.0*cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*sin(q4) + 0.5*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + 0.5*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
	T[2 - 1][2 - 1] = 0.75*cos(q1)*cos(q2)*cos(q5) - 0.75*cos(q5)*sin(q1)*sin(q2) - 0.25*cos(q1)*cos(q2)*cos(q5)*cos(q6) - 0.43301270189221932338186158537647*cos(q1)*cos(q3)*sin(q2)*sin(q4) - 0.43301270189221932338186158537647*cos(q1)*cos(q4)*sin(q2)*sin(q3) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*sin(q1)*sin(q4) - 0.43301270189221932338186158537647*cos(q2)*cos(q4)*sin(q1)*sin(q3) + 0.5*cos(q1)*cos(q2)*sin(q5)*sin(q6) + 0.25*cos(q5)*cos(q6)*sin(q1)*sin(q2) - 0.5*sin(q1)*sin(q2)*sin(q5)*sin(q6) + 0.75*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 0.75*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q4) - 0.43301270189221932338186158537647*cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q3) - 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) - 0.43301270189221932338186158537647*cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3) - 0.75*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.75*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - 0.5*cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - 0.25*cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - 0.5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) - 0.25*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) + 0.5*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 0.25*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 0.5*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6) + 0.25*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*sin(q5);
	T[2 - 1][3 - 1] = 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q5) - 0.43301270189221932338186158537647*cos(q5)*sin(q1)*sin(q2) + 0.43301270189221932338186158537647*cos(q1)*cos(q2)*cos(q5)*cos(q6) - 0.25*cos(q1)*cos(q3)*sin(q2)*sin(q4) - 0.25*cos(q1)*cos(q4)*sin(q2)*sin(q3) - 0.25*cos(q2)*cos(q3)*sin(q1)*sin(q4) - 0.25*cos(q2)*cos(q4)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q5)*sin(q6) - 0.43301270189221932338186158537647*cos(q5)*cos(q6)*sin(q1)*sin(q2) + 0.86602540378443864676372317075294*sin(q1)*sin(q2)*sin(q5)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + 0.75*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + 0.75*cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + 0.75*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) + 0.75*cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3) - 0.43301270189221932338186158537647*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + 0.43301270189221932338186158537647*cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + 0.86602540378443864676372317075294*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) + 0.43301270189221932338186158537647*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) - 0.86602540378443864676372317075294*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 0.43301270189221932338186158537647*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.86602540378443864676372317075294*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6) - 0.43301270189221932338186158537647*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*sin(q5);
	T[2 - 1][4 - 1] = 259.99712500078919711005658114022*cos(q1)*cos(q2) - 259.99712500078919711005658114022*sin(q1)*sin(q2) + 183.2484856184347722889290885445*cos(q1)*cos(q2)*cos(q5) + 410.0*cos(q1)*cos(q3)*sin(q2) + 410.0*cos(q2)*cos(q3)*sin(q1) - 183.2484856184347722889290885445*cos(q5)*sin(q1)*sin(q2) - 191.59281250197295776160899549723*cos(q1)*cos(q3)*sin(q2)*sin(q4) - 191.59281250197295776160899549723*cos(q1)*cos(q4)*sin(q2)*sin(q3) - 191.59281250197295776160899549723*cos(q2)*cos(q3)*sin(q1)*sin(q4) - 191.59281250197295776160899549723*cos(q2)*cos(q4)*sin(q1)*sin(q3) + 183.2484856184347722889290885445*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 183.2484856184347722889290885445*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - 183.2484856184347722889290885445*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 183.2484856184347722889290885445*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5);
	T[3 - 1][1 - 1] = 0.86602540378443864676372317075294*sin(q3)*sin(q4)*sin(q6) - 0.86602540378443864676372317075294*cos(q3)*cos(q4)*sin(q6) - 1.0*cos(q3)*cos(q5)*cos(q6)*sin(q4) - 1.0*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 0.5*cos(q3)*sin(q4)*sin(q5)*sin(q6) + 0.5*cos(q4)*sin(q3)*sin(q5)*sin(q6);
	T[3 - 1][2 - 1] = 0.43301270189221932338186158537647*sin(q3)*sin(q4) - 0.43301270189221932338186158537647*cos(q3)*cos(q4) - 0.43301270189221932338186158537647*cos(q3)*cos(q4)*cos(q6) - 0.75*cos(q3)*sin(q4)*sin(q5) - 0.75*cos(q4)*sin(q3)*sin(q5) + 0.43301270189221932338186158537647*cos(q6)*sin(q3)*sin(q4) + 0.5*cos(q3)*cos(q5)*sin(q4)*sin(q6) + 0.25*cos(q3)*cos(q6)*sin(q4)*sin(q5) + 0.5*cos(q4)*cos(q5)*sin(q3)*sin(q6) + 0.25*cos(q4)*cos(q6)*sin(q3)*sin(q5);
	T[3 - 1][3 - 1] = 0.25*sin(q3)*sin(q4) - 0.25*cos(q3)*cos(q4) + 0.75*cos(q3)*cos(q4)*cos(q6) - 0.43301270189221932338186158537647*cos(q3)*sin(q4)*sin(q5) - 0.43301270189221932338186158537647*cos(q4)*sin(q3)*sin(q5) - 0.75*cos(q6)*sin(q3)*sin(q4) - 0.86602540378443864676372317075294*cos(q3)*cos(q5)*sin(q4)*sin(q6) - 0.43301270189221932338186158537647*cos(q3)*cos(q6)*sin(q4)*sin(q5) - 0.86602540378443864676372317075294*cos(q4)*cos(q5)*sin(q3)*sin(q6) - 0.43301270189221932338186158537647*cos(q4)*cos(q6)*sin(q3)*sin(q5);
	T[3 - 1][4 - 1] = 191.59281250197295776160899549723*sin(q3)*sin(q4) - 191.59281250197295776160899549723*cos(q3)*cos(q4) - 410.0*sin(q3) - 183.2484856184347722889290885445*cos(q3)*sin(q4)*sin(q5) - 183.2484856184347722889290885445*cos(q4)*sin(q3)*sin(q5);
	T[4 - 1][1 - 1] = 0.0;
	T[4 - 1][2 - 1] = 0.0;
	T[4 - 1][3 - 1] = 0.0;
	T[4 - 1][4 - 1] = 1.0;

	/* define the current position*/
	double ret[4][1];
	ret[0][0] = 0.0;
	ret[1][0] = 0.0;
	ret[2][0] = 0.0;
	ret[3][0] = 0.0;

	/* Compute the position*/
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 4; k++) {
				ret[i][j] += T[i][k] * pt[k][j];
			}

		}
	}
	ret[0][0] = move_ptr->currPos[0];
	ret[1][0] = move_ptr->currPos[1];
	ret[2][0] = move_ptr->currPos[2];

}


void ik_RRR_arm(move* move_ptr, char* plane) {

	double px, py, a, b, c, th_a, th_b, th_c, delta, cos_tha, sin_tha, k;

	if (strcmp("xy", plane) != 0) {



	}
	else if (strcmp("zy", plane) != 0) {



	}
	else {
		printf("received plane: %s", plane);
		exit(1);
	}


	delta = (a*a) + (b*b) + 2.0 * a*b*cos(th_b);

	cos_tha = (px*(a + b*cos(th_b)) + py*b*sin(th_b)) / delta;
	sin_tha = (py*(a + b*cos(th_b)) - px*b*sin(th_b)) / delta;

	th_a = atan2(sin_tha, cos_tha);

	k = ((px*px) + (py*py) - (a*a) - (b*b)) / (2.0 * a*b);
	th_b = -1.0*acos(pow((k*k),0.5));

	th_c = acos((px - a*cos(th_a) - b*cos(th_a + th_b)) / c) - th_a - th_b;

}


int forward_xy_a(move* move_ptr) {

	double c;

	double ang = (3.141592 / 2.0) - move_ptr->currAng[2];
	double H = c / cos(move_ptr->currAng[3]);


	return 0;

}


void move_target(info * info_ptr, move * move_ptr, char direction)
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
	if (direction == 'w') { position[1] += (simxFloat)(0.05); }
	if (direction == 's') { position[1] -= (simxFloat)(0.05); }
	if (direction == 'a') { position[0] += (simxFloat)(0.05); }
	if (direction == 'd') { position[0] -= (simxFloat)(0.05); }
	printf("%f %f %f\n", position[0], position[1], position[2]); fflush(stdout);


	simxSetObjectPosition(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &position, simx_opmode_oneshot);

}

void get_position(info* info_ptr, simxFloat* startPosition, int relativeHandle) {

	;
}
