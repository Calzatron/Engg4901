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
#include <time.h>


/*	Global Definitions	*/
#define BUFSIZE 4096 


HANDLE  hConsoleOut;                 // Handle to the console   
HANDLE  hRunMutex;                   // "Keep Running" mutex   
HANDLE  hScreenMutex;                // "Screen update" mutex  
int     ThreadNr;                    // Number of threads started   
CONSOLE_SCREEN_BUFFER_INFO csbiInfo; // Console information 

HANDLE jointMutexes[5];

struct jointThreads
{
	info *information_ptr;
	move *movement_ptr;
	HANDLE threadNumber;

};

/*	Function Definitions	*/
info* makeInfo(void); 
move* makeMove(void);
void get_object_names_vrep(info* info_ptr);
void write_object_info(info* info_ptr, char* filename);
void get_command(info* info_ptr, move* move_ptr);
void get_joint_angles_vrep(info* info_ptr, move* move_ptr);
void initial_arm_config_vrep(info* info_ptr, move* move_ptr);
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang, bool getJointAngles);
void interpret_command_fk(info* info_ptr, move* move_ptr);
void interpret_command_ik(info* info_ptr, move* move_ptr, bool commandLine);
void CreateChildProcess(void);
void WriteToPipe(void);
void ReadFromPipe(void);
void move_target_vrep(info* info_ptr, move* move_ptr, char direction, float duty);
void read_object_info(info* info_ptr, char* filename);
void get_position_vrep(info* info_ptr, float* position, int handle);
void get_orientation_vrep(info* info_ptr, float* orientation, int handle, int relativeHandle);
void get_world_position_vrep(info* info_ptr, float* position, int handle);
void set_world_position_vrep(info* info_ptr, float* position, int objectHandle);
void set_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void pause_communication_vrep(info* info_ptr, int status);
void move_tip_vrep(info* info_ptr, move* move_ptr, char command, float duty);
void stream_joints(struct jointThreads *threadStruct);
void handle_joint_threads(info* info_ptr, move* move_ptr);
double current_angle(move* move_ptr, int jointNum);

/*	Program Functions	*/


/*	Determine the mode of the Jaco arm in Vrep from input commands	*/
void initialise_program(info* info_ptr, char argc, char** argv) {
	
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
	info_ptr->sceneMode = malloc(sizeof(char) * 3);
	info_ptr->programMode = malloc(sizeof(char) * 3);
	for (int j = 0; j < i; j++) {
		if (((inputBuffer[j] == 'i') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'I') && (inputBuffer[j + 1] == 'K'))) {
			strcpy(info_ptr->sceneMode, "ik");
			strcpy(info_ptr->programMode, "ik");
		}
		else if (((inputBuffer[j] == 'f') && (inputBuffer[j + 1] == 'k')) ||
			((inputBuffer[j] == 'F') && (inputBuffer[j + 1] == 'K'))) {
			
			strcpy(info_ptr->sceneMode, "fk");

			/*	The scene was specified to be in FK mode
			*	Determine if the program inputs will be for FK or IK	*/
			printf("Would you like ik inputs with a fk VREP scene [Y/n]>>");
			char c = getchar();

			if ((c == '\n') || (c == '\r')) {
				/*	Choose ik by default (enter)	*/
				strcpy(info_ptr->programMode, "ik");
			}

			char extraBuffer[20];
			i = 0;
			while ((c != '\n') && (i < 10)) {
				/*	store input	*/
				extraBuffer[i] = c;
				c = getchar();
				++i;
			}
			if ((extraBuffer[0] == 'Y') || (extraBuffer[0] == 'y')) {
				strcpy(info_ptr->programMode, "ik");
			}
			else if ((extraBuffer[0] == 'n') || (extraBuffer[0] == 'N')) {
				strcpy(info_ptr->programMode, "fk");
			}
			else {
				strcpy(info_ptr->programMode, "ik");
			}
			printf("\n%s selected\n", info_ptr->programMode);
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

	/*	store all object handles	*/
	info_ptr->objectHandles = objectHandles;

	#ifdef DEBUG
		printf("DEBUG>> objectHandles[40] %d \n", info_ptr->objectHandles[40]);
	#endif // DEBUG

	
    //printf("DEBUG>> objectHandles[40] %d \n", info_ptr->objectHandles[40]);

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
		#ifdef DEBUG
				printf("File path: %s exists: %d true: %d\n", objectFilePath, exists, true);
		#endif // DEBUG
	
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
	printf("Wait, getting information... ");
	/* Retrieve all angles of joints to be used in kinematics
	*	Store the positions in move_ptr->currAng[i] for the i'th joint */
	initial_arm_config_vrep(info_ptr, move_ptr);

	/* Create a child process to retreive button presses and joystick
	*	positions from the gaming controller	*/
	CreateChildProcess();

	/*	Initialise Joystick Mutex	*/
	initialise_joystick_mutex();

	#ifdef DEBUG
		printf("Child Process Created\n");
	#endif // DEBUG

	/*	Begin a thread for processing the commands from either
	*	control inputs or the joystick	*/
	hConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	hScreenMutex = CreateMutex(NULL, FALSE, NULL);		// Cleared   
	hRunMutex = CreateMutex(NULL, TRUE, NULL);			// Set   
	ThreadNr = 0;										//initialise threadcount
	ThreadNr++;											// increment threadcount for command thread
	_beginthread(ReadFromPipe, 0, &ThreadNr);			// direct the thread to their new home
	
	
	ThreadNr++;											// increment threadcount for command thread
	_beginthread(add_to_buffer, 0, &ThreadNr);			// direct the thread to their new home
	
	
	////////////////////////********* Under Construction **********//////////////////
	#ifdef DATA_STREAMING

		handle_joint_threads(info_ptr, move_ptr);

	#endif //DATA_STEAMING
	/////////////////////////////////////////////////////////////////////////////////


	while (1) {
		//printf(".");
		/*	Need to work out how to choose between command or joystick based control	*/
		if ((strcmp(info_ptr->sceneMode, "ik") == 0) && (joystickEnabled())) {

			//printf("ik and joystickEn\n"); fflush(stdout);
			/*	A joystick was found, get inputs from this	*/
			if (joystick_input_available()) {
				/*	A joystick command is available to action on	*/
				joystick_get_char(info_ptr);

				interpret_command_ik(info_ptr, move_ptr, false);
			}
		}
		else {

			//if (strcmp(info_ptr->sceneMode, "fk") == 0) {
			if (strcmp(info_ptr->programMode, "fk") == 0) {
				/*	Instead of getting input from joystick, get from command line input	*/
				#ifdef DEBUG
					printf("commandline ");
				#endif // DEBUG

				get_command(info_ptr, move_ptr);
			}
			else {
				/*	Program Mode is ik but scene is fk... Input can be joystick or commandline
				*	'wasd' 
				**/
				if (joystickEnabled()) {
					if (joystick_input_available()) {
						/*	A joystick command is available to action on	*/
						joystick_get_char(info_ptr);
						interpret_command_ik(info_ptr, move_ptr, false);
					}
				}
				else {
					#ifdef DEBUG
						printf("commandline ik");
					#endif // DEBUG
					get_command(info_ptr, move_ptr);
				}
				//printf("entering ik\n"); fflush(stdout);
				//inverse_kinematics(move_ptr, info_ptr);
			}
		}
	}


    extApi_sleepMs(2000);

    simxFinish(-1); // close all open connections

    return 0;
}


void handle_joint_threads(info* info_ptr, move* move_ptr) {

	/*	Create a thread for each joint	*/
	for (int i = 0; i < 5; i++) {

		/*	Create the mutex for this joint	*/
		jointMutexes[i] = CreateMutex(NULL, FALSE, NULL);

		if (jointMutexes[i] == NULL) {
			printf("CreateMutex error: %d\n", GetLastError());
			continue;
		}

		//printf("before struct: %d\n", info_ptr->clientID);

		/*	Copy the struct information	*/
		struct jointThreads threadStruct;

		threadStruct.information_ptr = info_ptr;
		threadStruct.movement_ptr = move_ptr;
		threadStruct.threadNumber = (HANDLE)(ThreadNr);

		//printf("before thread: %d\n", threadStruct.information_ptr->clientID);

		//printf("this is angle before: %d, %f\n", info_ptr->jacoArmJointHandles[0], current_angle(move_ptr, 0));

		/*	Generate a new thread to stream data	*/
		ThreadNr++;												// increment threadcount for command thread
		_beginthread(stream_joints, 0, &threadStruct);			// direct the process to their new home
		clock_t current_time = clock();
		while (current_time + 2000 > clock()) {
			;
		}
	}

	#ifdef THREAD_DEBUG
		while (1) {
			for (int j = 0; j < 50000; j++) {

				printf("this is angle after: %d, %f\n", info_ptr->jacoArmJointHandles[0], current_angle(move_ptr, 0));
			}
		}
	#endif // THREAD_DEBUG

}


void stream_joints(struct jointThreads *threadStruct) {

	/*	Regain the information from the input struct	*/
	int threadNumber = (int)(threadStruct->threadNumber) - 1;
	info* info_ptr = threadStruct->information_ptr;
	move* move_ptr = threadStruct->information_ptr;

	//printf("thread %d\n", threadNumber);

	/*	Generate a new stream to receive data from	*/
	int streamingID = simxStart((simxChar*)"127.0.0.1", 19999 - (threadNumber *1000), true, true, 5000, 50);

	if (streamingID != -1) {
		printf("Successfully connected to VREP: %d %d %d\n", info_ptr->clientID, streamingID, threadNumber);
	}
	else {
		printf("Connection failed %d %d %d\n", info_ptr->clientID, streamingID, threadNumber);
		_endthread();
	}
	
	/*	Set-up streaming of joint data	*/
	simxFloat position;

	simxGetJointPosition(streamingID, info_ptr->jacoArmJointHandles[threadNumber - 1], &position, simx_opmode_streaming);

	#ifdef THREAD_DEBUG
			printf(".%d\n", info_ptr->jacoArmJointHandles[threadNumber - 1]);
	#endif // DEBUG

	/*	When connected, stream data	*/
	while (simxGetConnectionId(streamingID) != -1) { // while we are connected to the server..

		// Fetch the newest joint value from the inbox (func. returns immediately (non-blocking)):
		if (simxGetJointPosition(streamingID, info_ptr->jacoArmJointHandles[threadNumber - 1], &position, simx_opmode_buffer) == simx_return_ok) {

			/*	Received new value, toggle mutex to enter value	*/
			DWORD bufferWaitResult = WaitForSingleObject(jointMutexes[threadNumber - 1], INFINITE);  // no time-out interval

			if (bufferWaitResult == WAIT_ABANDONED) {
				printf("Mutex in indeterminate state");
			}
			else if (bufferWaitResult == WAIT_OBJECT_0) {

				// here we have the newest joint position in variable jointPosition!
				move_ptr->currAng[threadNumber - 1] = 0;

				move_ptr->currAng[threadNumber - 1] = (double)position - 3.141592;
				if (move_ptr->currAng[threadNumber - 1] < 0) {
					move_ptr->currAng[threadNumber - 1] += 2.0 * 3.141592;
				}

				#ifdef THREAD_DEBUG
					printf("this is angle: %d, %f, %f\n", info_ptr->jacoArmJointHandles[threadNumber - 1], position, move_ptr->currAng[threadNumber - 2]);
				#endif // DEBUG


				if (!ReleaseMutex(jointMutexes[threadNumber - 1]))
				{
					// Handle error.
					printf("Could not release Mutex %d\n", GetCurrentThreadId());
				}
			}
		}
		else {
			// once you have enabled data streaming, it will take a few ms until the first value has arrived. So if
			// we landed in this code section, this does not always mean we have an error!!!
		}
	} printf("%d exited streaming\n", threadNumber);

	/*	No reason to stay in this thread, exit	*/
	_endthread();
}



/* Initialize a struct to store all the Vrep scene info */
info* makeInfo(void){
    
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}

/* Initialize a struct to store all the Vrep scene info */
move* makeMove(void) {
	
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


/*	Prompts ready to receive command from stdin
*	stores the input in the response buffer
*	and sends it for interpreting	*/
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

	#ifdef DEBUG
		printf("received: %s\n", info_ptr->response);
	#endif // DEBUG

	/*	Command received, send it to get interpretted	*/
	if ((strcmp(info_ptr->sceneMode, "fk") == 0) && (strcmp(info_ptr->programMode, "fk") == 0)) {
		/*	The scene is in fk and inputs are fk (move a joint a given angle)	*/
		interpret_command_fk(info_ptr, move_ptr);
		//free(info_ptr->response); // not free'd yet
	}
	else if ((strcmp(info_ptr->sceneMode, "ik") == 0) || (strcmp(info_ptr->programMode, "ik") == 0)) {
		/*	The scene is in IK and the inputs are IK or the scene is in FK and inputs are IK
		*	so the inputs are interpreted the same but response will be different 
		*	(position of tip to be altered, determined by input, and arm follow)
		*/
		interpret_command_ik(info_ptr, move_ptr, true); 
	}

	#ifdef DEBUG
		printf("finished interpret_command_ik\n");
	#endif // DEBUG

}

/*
	Interprets the input command in info_ptr->response given from a joystick or
	commandline input
*/
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

	#ifdef DEBUG
		printf("	*Duty:	%f\n", duty);
		//pritnf("	programMode %s\n")
	#endif // DEBUG


	if (strcmp(info_ptr->sceneMode, "ik") == 0) {
		#ifdef DEBUG
				printf("going to move_target_vrep\n"); fflush(stdout);
		#endif // DEBUG
		if (info_ptr->response[0] == 'w') { move_target_vrep(info_ptr, move_ptr, 'w', duty); }
		if (info_ptr->response[0] == 'a') { move_target_vrep(info_ptr, move_ptr, 'a', duty); }
		if (info_ptr->response[0] == 's') { move_target_vrep(info_ptr, move_ptr, 's', duty); }
		if (info_ptr->response[0] == 'd') { move_target_vrep(info_ptr, move_ptr, 'd', duty); }
		if (info_ptr->response[0] == '-') { move_target_vrep(info_ptr, move_ptr, '-', duty); }
		if (info_ptr->response[0] == '+') { move_target_vrep(info_ptr, move_ptr, '+', duty); }
		
	}
	else {
		/*	FK scene but given IK input
		*	Begin control loop and kinematics for moving joints	*/////////////////////////////////////////////////////////////////
		#ifdef DEBUG
			printf("going to get_joint_angles_vrep\n"); fflush(stdout);
		#endif // DEBUG

		#ifndef DATA_STREAMING
			get_joint_angles_vrep(info_ptr, move_ptr);
		#endif // !DATA_STREAMING

		#ifdef DEBUG
			printf("going to move_tip_vrep\n"); fflush(stdout);
		#endif // DEBUG

		
		if (info_ptr->response[0] == 'w') { move_tip_vrep(info_ptr, move_ptr, 'w', duty); }
		if (info_ptr->response[0] == 'a') { move_tip_vrep(info_ptr, move_ptr, 'a', duty); }
		if (info_ptr->response[0] == 's') { move_tip_vrep(info_ptr, move_ptr, 's', duty); }
		if (info_ptr->response[0] == 'd') { move_tip_vrep(info_ptr, move_ptr, 'd', duty); }
		if (info_ptr->response[0] == '-') { move_tip_vrep(info_ptr, move_ptr, '-', duty); }
		if (info_ptr->response[0] == '+') { move_tip_vrep(info_ptr, move_ptr, '+', duty); }
		if (info_ptr->response[0] == 'p') { move_tip_vrep(info_ptr, move_ptr, 'p', duty); }
		if ((info_ptr->response[0] == 'f') && (info_ptr->response[1] == 'k')) {
			/*	was asked to get the joint angles using the classic DH parameters and FK	*/
			fk_classic(move_ptr, info_ptr);								// UPDATED
		}
		
	}

	/*	End of interpreting, free response	*/
	free(info_ptr->response);
}


/*
	Interprets commands in info_ptr->response (from stdin) given
	that the scene and program are in fk mode
*/
void interpret_command_fk(info* info_ptr, move* move_ptr) {
	
	#ifdef DEBUG
		printf("interpret_command\n");
	#endif // DEBUG
	//printf("interpret_command\n");


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
		fk_classic(move_ptr, info_ptr);								// NEEDS UPDATING
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
	move_joint_angle_vrep(info_ptr, move_ptr, jointNum, ang, true);
	
	/*	end of interpreting, free the response	*/	
	free(info_ptr->response);
}


/*  Retrieves object names for corresponding object handle,
*  stored in info struct
*
*  Calls a custom function in VRep main script, which takes in an
*  object handle and returns the object's name.
*/
void get_object_names_vrep(info* info_ptr){
    
    int replySize[1] = {1};
    
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//for (int j = 0; j < info_ptr->objectCount; j++) {
	
	//	printf("objectHandle[%d]: %d\n", j, info_ptr->objectHandles[j]);
	//}
	//exit(0);
	///////////////////////////////////////////////////////////////////////////////////////////////////


    for(int i = 0; i < info_ptr->objectCount; ++i){
        int ret = 0;
        simxChar* replyData;
		//printf("%d		", info_ptr->objectHandles[i]);
		simxInt num = i;//info_ptr->objectHandles[i];
        info_ptr->isJoint[i] = 0;
        ret = simxCallScriptFunction(info_ptr->clientID, "", sim_scripttype_mainscript,
                "get_object_names", 1, &num, 0, NULL, 0, NULL, 0, NULL, 
                NULL, NULL, NULL, NULL, &replySize, &replyData, NULL, NULL,
                simx_opmode_blocking);
        if (ret != simx_return_ok){
            printf("ret not ok %d\n", i);
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
				info_ptr->targetHandle = info_ptr->objectHandles[i];
				printf("Target is %d indexed at: %d\n", info_ptr->objectHandles[i], i); //i+1;
			}
            token = strtok(NULL, underScore);
            if (count == 5){
                info_ptr->isJoint[i] = 1;
            }
        }
    }
    printf("Got object names\n");
}


/* Puts the arm into a starting pos, straight up, by rotating joints 4 and 5
*  the joint handles are also found, and angles of each using get_joint_angles.
*  When the program mode is ik, moves joint 4 so the tip and base are aligned */
void initial_arm_config_vrep(info* info_ptr, move* move_ptr) {
	
	#ifdef DEBUG
		printf("initial_arm_config_vrep\n");
	#endif // DEBUG
	
	int i = 0;
	int num = 0;
	info_ptr->jacoArmJointHandles = malloc(sizeof(int) * 6);
	
	#ifdef DEBUG
		printf("jacoarmjointhandles malloced %d\n", info_ptr->objectCount);
	#endif // DEBUG

	while (num < 6) {

		if (info_ptr->isJoint[i]) {
			info_ptr->jacoArmJointHandles[num] = info_ptr->objectHandles[i];
			#ifdef DEBUG
						printf("joint handle: %d\n", info_ptr->jacoArmJointHandles[num]);

			#endif // DEBUG

			++num;
		}
		++i;
		if (i > info_ptr->objectCount) {
			printf("i: %d\n", i);
			exit(1);
		}
	}

	#ifndef DATA_STREAMING
		/*	Create a mutex for each joint	*/
		for (int i = 0; i < 5; i++) {

			jointMutexes[i] = CreateMutex(NULL, FALSE, NULL);

			if (jointMutexes[i] == NULL) {
				printf("CreateMutex error: %d\n", GetLastError());
				continue;
			}
		}

		get_joint_angles_vrep(info_ptr, move_ptr);
	#endif // !DATA_STREAMING

	
	if (strcmp(info_ptr->programMode, "fk") == 0) {
		/*	set the arm into the upright position	*/
		//int ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[5 - 1], 5, simx_opmode_oneshot_wait);
		//ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[5 - 1], 25, simx_opmode_oneshot_wait);
		//ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[5 - 1], 0, simx_opmode_oneshot_wait);

		//ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 5, simx_opmode_oneshot_wait);
		//ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 25, simx_opmode_oneshot_wait);
		//ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 0, simx_opmode_oneshot_wait);
	}
	else if (strcmp(info_ptr->programMode, "ik") == 0) {
		if ((move_ptr->currAng[0] < 0.1) && (move_ptr->currAng[0] > -0.1) && (move_ptr->currAng[3] < 3.2) && (move_ptr->currAng[3] > 3.1)){
			move_joint_angle_vrep(info_ptr, move_ptr, 1, -45, false);
			move_joint_angle_vrep(info_ptr, move_ptr, 4, 90, false);
			move_joint_angle_vrep(info_ptr, move_ptr, 2, 10, false);
			move_joint_angle_vrep(info_ptr, move_ptr, 3, -30, false);
		}
	}

	/*	Gets and stores the position of the jaco's base element relative to the world frame	*/
	simxFloat position[3];
	simxInt handle;
	for (int j = 0; j < info_ptr->objectCount; j++) {
		if (info_ptr->isJoint[j]) {
			handle = (simxInt)(info_ptr->objectHandles[j]);
			simxGetObjectPosition(info_ptr->clientID, handle, -1, &position, simx_opmode_blocking);
			info_ptr->armPosition = malloc(sizeof(simxFloat) * 3);
			info_ptr->armPosition[0] = position[0];
			info_ptr->armPosition[1] = position[1];
			info_ptr->armPosition[2] = position[2];
			//printf("Jaco arm %d position: %f %f %f\n", handle, info_ptr->armPosition[0], info_ptr->armPosition[1], info_ptr->armPosition[2]);
			break;
		}
	}
}


/*  Writes all the object handles and corresponding names to filename
*  if filename already exists, it clears the file before writing
*  filename is either an input argument or by default called "objects"
*/
void write_object_info(info* info_ptr, char* filename){

    FILE* object_fp = fopen(filename, "w+");
    if (object_fp == NULL){
        fprintf(stdout, "Failed to generate file\n");
        exit(1);
    }
    
    for (int i = 0; i < info_ptr->objectCount; i++){
        if (info_ptr->isJoint[i]){ 
            char line[256];
            sprintf(line, "%d %s\n", info_ptr->objectHandles[i], info_ptr->objectNames[i]);
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
	#ifdef DEBUG
		printf("This is the target's object handle: %d\n", info_ptr->targetHandle);
	#endif // DEBUG
	
}


/*	Updates the position vector with relativeHandle's orientation relative to the base of the arm	*/
void get_orientation_vrep(info* info_ptr, float* orientation, int handle, int relativeHandle) {
	
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


/*	Updates the position vector with relativeHandle's position relative to the base of the arm.
	Good for x,y coordinates, but not z as the base joint has an offset height
*/
void get_position_vrep(info* info_ptr, float* position, int handle) {
		 
	int jointHandle1 = info_ptr->jacoArmJointHandles[0];
	/*for (int i = 0; i < info_ptr->objectCount; i++) {
		if (info_ptr->isJoint[i]) {
			jointHandle1 = i;
		}
	}*/
	simxFloat pos[3];
	// get position of handle relative to jaco_joint_1
	simxGetObjectPosition(info_ptr->clientID, handle, jointHandle1, &pos, simx_opmode_blocking);

	position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];

}

/*
	Updates position vector with the world coordinates of the object given by handle
*/
void get_world_position_vrep(info* info_ptr, float* position, int handle) {

	simxFloat pos[3];
	simxGetObjectPosition(info_ptr->clientID, handle, -1, &pos, simx_opmode_blocking);
	position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];
	#ifdef DEBUG
		printf("world position: %f %f %f\n", position[0], position[1], position[2]);
	#endif // DEBUG

}


/*
	Sets the position of an object given a position vector and object handle,
	relative to the world frame.
*/
void set_world_position_vrep(info* info_ptr, float* position, int objectHandle) {

	simxFloat pos[3];
	pos[0] = position[0];
	pos[1] = position[1];
	pos[2] = position[2];
	#ifdef DEBUG
		printf("Sending target %d to %f %f %f\n", objectHandle, pos[0], pos[1], pos[2]);
	#endif // DEBUG

	simxSetObjectPosition(info_ptr->clientID, objectHandle, -1, &pos, simx_opmode_blocking);
}


/* Updates info struct with all jaco arm joint current angles
*  joints 4 and 5 are initially adjusted to take on FK values,
*  the other joints are offset by 180 degrees */
void get_joint_angles_vrep(info* info_ptr, move* move_ptr) {
	

	#ifdef DEBUG
		printf("get_joint_angles_vrep\n");
	#endif // DEBUG

	int count = 0;
	while (count < 6) {


		#ifdef DATA_STREAMING


		/*	Received new value, toggle mutex to enter value	*/
		DWORD bufferWaitResult = WaitForSingleObject(jointMutexes[count], INFINITE);  // no time-out interval

		if (bufferWaitResult == WAIT_ABANDONED) {
			printf("Mutex in indeterminate state");
		}
		else if (bufferWaitResult == WAIT_OBJECT_0) {
		#endif // DATA_STREAMING

			#ifdef DEBUG
				printf(".%d\n", info_ptr->jacoArmJointHandles[count]);
			#endif // DEBUG

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

			#ifdef DEBUG
				printf("this is angle: %d, %f, %f\n", info_ptr->jacoArmJointHandles[count], position, move_ptr->currAng[count]);
			#endif // DEBUG

			++count;

		#ifdef DATA_STREAMING
			/*	Release Mutex for this joint	*/
			if (!ReleaseMutex(jointMutexes[count]))
			{
				// Handle error.
				printf("Could not release Mutex %d\n", GetCurrentThreadId());
			}
		}
		#endif // DATA_STREAMING
	}
}


/*	takes in a joint number between 1 and 6, these are translated into
	object handles, and the arm is moved via external command
	ang is in degrees and represents the change in angle to move the joint
	specified by jointNum */
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang, bool getJointAngles) {
	
	if (getJointAngles) {
		#ifndef DATA_STREAMING
			get_joint_angles_vrep(info_ptr, move_ptr);
		#endif // !DATA_STREAMING
	}

	double jointAngle = current_angle(move_ptr, jointNum - 1) + (ang*3.141592 / 180.0);
	if ((jointNum != 4) && (jointNum != 5)) {
		jointAngle += 3.141592;
	}
	jointAngle = fmod(jointAngle, 2 * 3.141592);
	#ifdef DEBUG
		printf("moving joint %d : %f : %f\n", jointNum, jointAngle, ang);
	#endif // DEBUG

	/*	Move the arm	*/
	int ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 50, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], jointAngle, simx_opmode_oneshot_wait);
}


/*	takes in a joint number between 1 and 6, these are translated into
	object handles, and the arm is moved via external command
	ang is in radians and represents the true position of the joint.
	This differs to move_joint_angle_vrep which moves a change in angle (degrees) */
void set_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang) {


	double jointAngle = ang;
	if ((jointNum != 4) && (jointNum != 5)) {
		jointAngle += 3.141592;
	}
	//jointAngle = fmod(jointAngle, 2 * 3.141592);
	//printf("moving joint %d : %f : %f\n", jointNum, jointAngle, ang);

	if (jointAngle < -0.1) {
		jointAngle = 2 * 3.141592 + jointAngle;
	}
	else if (jointAngle > 2 * 3.141592) {
		jointAngle = fmod(jointAngle, 2 * 3.141592);
	}

	printf("moving joint %d : %f : %f\n", jointNum, jointAngle, ang);

	//return;

	int ret;
	if (jointNum == 2) {
		int velocity = (25 * 410) / 207;
		ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], velocity, simx_opmode_oneshot);
	}
	else if (jointNum == 3) {
		int velocity = (25 * 207) / 410;
		ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], velocity, simx_opmode_oneshot);
	}
	else {
		ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 25, simx_opmode_oneshot);
	}
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], 25, simx_opmode_oneshot);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[jointNum - 1], jointAngle, simx_opmode_oneshot);
}


/*	move's the target for the arm tip to follow
	the movement is specified by direction, with a displacement scaled by duty	*/
void move_target_vrep(info * info_ptr, move * move_ptr, char direction, float duty)
{
	
	#ifdef DEBUG
		printf("move_target(), %d\n", info_ptr->targetHandle); fflush(stdout);
	#endif // DEBUG
	
	/*	Get the target's position relative to the base of the arm	*/
	simxFloat position[3];
	simxFloat orientation[3];
	simxGetObjectPosition(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &position, simx_opmode_blocking);
	simxGetObjectOrientation(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &orientation, simx_opmode_blocking);
	
	#ifdef DEBUG
		printf("%f %f %f	%f %f %f\n", position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]); fflush(stdout);
	#endif // DEBUG

	/*	Determine new position to move from input	*/
	if (direction == 'w') {			//position[1] += (simxFloat)(0.01 * duty); }
		/*	moves the tip away from the arm's base	*/

		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2) + (0.03*duty);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle);
		position[1] = hype * sinf(angle);

	}
	else if (direction == 's') {	//position[1] -= (simxFloat)(0.01 * duty); }
		/*	moves the tip towards the arm's base	*/

		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2) - (0.03*duty);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle);
		position[1] = hype * sinf(angle);
		
	}
	else if (direction == 'a') {	//position[0] += (simxFloat)(0.01 * duty); }
		/*	moves the arm clockwise radially from the centre position of the arm	*/
		
		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle + (duty*4.0*3.141592 / 180.0));
		position[1] = hype * sinf(angle + (duty*4.0*3.141592 / 180.0));

	}
	else if (direction == 'd') {	//position[0] -= (simxFloat)(0.01 * duty); }
		/*	moves the arm anti-clockwise radially from the centre position of the arm	*/

		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle - (duty*4.0*3.141592 / 180.0));
		position[1] = hype * sinf(angle - (duty*4.0*3.141592 / 180.0));
	
	}
	else if (direction == '+') { position[2] += (simxFloat)(0.02 * duty); }
	else if (direction == '-') { position[2] -= (simxFloat)(0.02 * duty); }
	
	#ifdef DEBUG
		printf("%f %f %f\n", position[0], position[1], position[2]); fflush(stdout);
	#endif // DEBUG

	/*	write to VREP the new position	*/
	simxSetObjectPosition(info_ptr->clientID, info_ptr->targetHandle, sim_handle_parent, &position, simx_opmode_oneshot);

}


/*	Determines the next position the tip should be in from the commands
	Command is w,s,a,d,-,+
	Duty is a decimal between 0->1 that scales the next position 
*/
void move_tip_vrep(info* info_ptr, move* move_ptr, char command, float duty) {
	
	#ifdef DEBUG
		printf("in move_tip_vrep %c %f\n", command, duty); fflush(stdout);
	#endif // DEBUG
	
	/*	Get the position of the tip in world frame	*/
	simxFloat position[3];
	simxGetObjectPosition(info_ptr->clientID, info_ptr->targetHandle - 1, -1, &position, simx_opmode_blocking);

	/*	Get the position of the base in the world frame	*/
	simxFloat worldPosition[3];
	worldPosition[0] = info_ptr->armPosition[0];
	worldPosition[1] = info_ptr->armPosition[1];
	worldPosition[2] = info_ptr->armPosition[2];
	
	/*	Get the position of the tip relative to the base, without z	*/
	position[0] -= worldPosition[0];
	position[1] -= worldPosition[1];

	/*	Get the position of joint 5 relative to the base	*/
	simxFloat joint5Position[3];
	simxGetObjectPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[5-1], -1, &joint5Position, simx_opmode_blocking);
	joint5Position[0] -= worldPosition[0];
	joint5Position[1] -= worldPosition[1];

	/*	Calculate the xy magnitude of joint 5 and that of the tip	*/
	float joint5Mag = joint5Position[0] * joint5Position[0] + joint5Position[1] * joint5Position[1];
	float tipMag = position[0] * position[0] + position[1] * position[1];

	/*	Ensure the tip is lower than joint 5 and pointing outwards form the arm	*/
	if ((tipMag < joint5Mag) && (joint5Position[2] < position[2])) {
		/*	Need to turn the wrist around	*/	
		move_joint_angle_vrep(info_ptr, move_ptr, 4, 180, false);
		/*	Wait until it has moved into the right position	*/
		while ((tipMag < joint5Mag) && (joint5Position[2] < position[2])) {
			for (int wait = 0; wait < 200; wait++) { ; }
			/*	Update tip position and check against criteria	*/	
			simxGetObjectPosition(info_ptr->clientID, info_ptr->targetHandle - 1, -1, &position, simx_opmode_blocking);
			tipMag = position[0] * position[0] + position[1] * position[1];
			printf("...");
		}
	}

	/*	Calculate the position of the tip relative to +x and +y from world coordinates	*/	
	double tipAngle = (double)(atan2f(position[1], position[0]));
	tipAngle = (3.141592 - tipAngle);
	if (tipAngle < 0) {
		tipAngle = 2 * 3.141592 + tipAngle;
	}
	//fmod(tipAngle, 3.141592));

	
	/*	Get the current angle of the base	*/
	double jointAngle = current_angle(move_ptr, 0);	// = fmod(move_ptr->currAng[0], 3.141592);
	if (jointAngle < 0) {
		jointAngle = 2 * 3.141592 + jointAngle;
	}
	//jointAngle = fmod(jointAngle, 3.141592);

	#ifdef DEBUG
		printf("tipAngle: %f,	jointAngle: %f,		base Angle: %f\n", tipAngle, current_angle(move_ptr, 3), jointAngle);
	#endif // DEBUG
		
	double changeAngle;

	if (command == 'p') {
		/*	Print out the orientation of the tip and base joint	*/
		changeAngle = 180.0*(jointAngle - tipAngle) / 3.141592;
		/*	Correct for zero crossing in both directions	*/
		if (((jointAngle > 3.0*3.141592 / 2.0) && (tipAngle < 3.141592 / 2.0))
			|| ((jointAngle < 0.0) && (tipAngle < 3.141592 / 2.0))) {
			changeAngle = changeAngle - 360.0;
		}
		else if (((tipAngle > 3.0*3.141592 / 2.0) && (jointAngle < 3.141592 / 2.0))
			|| ((tipAngle < 0.0) && (jointAngle < 3.141592 / 2.0))) {
			changeAngle = changeAngle + 360.0;
		}
		printf("tipAngle: %f,	base Angle: %f		change: %f\n", tipAngle, jointAngle, changeAngle);
		return;
	}

	if ((command != 'a') && (command != 'd')) {
		/*	Ignore this correction if only the base joint is moving	*/

		tipAngle = fmod(tipAngle, 3.141592);

		int notAligned = 1;
		while (notAligned) {

			/*	Loop until the tip is aligned with the base orientation	*/
			changeAngle = 180.0*(fmod(jointAngle, 3.141592) - tipAngle) / 3.141592;
			/*	Correct for zero crossing in both directions	*/	
			if (((jointAngle > 3.0*3.141592 / 2.0) && (tipAngle < 3.141592/2.0)) 
				|| ((jointAngle < 0.0) && (tipAngle < 3.141592 / 2.0))) {
				changeAngle = changeAngle - 360.0;
			} else if (((tipAngle > 3.0*3.141592 / 2.0) && (jointAngle < 3.141592 / 2.0))
				|| ((tipAngle < 0.0) && (jointAngle < 3.141592 / 2.0))) {
				changeAngle = changeAngle + 360.0;
			}

			if (fabs(changeAngle) > 3) {
				
				#ifdef DEBUG
					printf("Correction: changeAngle		%f\n", changeAngle);
					printf("Correction: tipAngle		%f\n", tipAngle);
					printf("Correction: basejointAngle	%f\n", jointAngle);
				#endif // DEBUG

				move_joint_angle_vrep(info_ptr, move_ptr, 4, 0.8 * changeAngle, true);
				for (int i = 0; i < 700; i++) {
					; // delay and wait for the arm to move
				}

				/*	Determine the position fo Joint4, it can get stuck if it is at 0 or pi
				*	Because the error switches between +-MaxError	*/
				simxFloat joint4Angle;
				int ret = simxGetJointPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], &position, simx_opmode_blocking);

				joint4Angle = fmodf(joint4Angle, 3.141592);
				if (joint4Angle < 0.3 || joint4Angle > 6.0) {
					/*	The arm is likely to be or is about to be stuck in correction
					*	move the arm to a defined position and exit	*/
					//move_joint_angle_vrep(info_ptr, move_ptr, 4, 90, false);
					//break;
				}

				/*	Get the position of the tip and recalculate the angle to correct for	*/
				simxGetObjectPosition(info_ptr->clientID, info_ptr->targetHandle - 1, -1, &position, simx_opmode_blocking);
				position[0] -= worldPosition[0];
				position[1] -= worldPosition[1];
				tipAngle = (double)(atan2f(position[1], position[0]));
				/*	Modulate the tip angle as it doesn't matter which frame it is in	*/
				tipAngle = (3.141592 - tipAngle);

				if (tipAngle < 0) {
					/*	Ensure the angle is positive to the change in angle is accurate in direction	*/
					tipAngle = 2 * 3.141592 + tipAngle;
				}
				tipAngle = fmod(tipAngle, 3.141592);
				

			}
			else {
				/*	The angle has been corrected, the joints are aligned	*/
				notAligned = 0;
				break;
			}
		}

	}

	/*	Display the current tip position	*/	
	#ifdef DEBUG
		printf("move_tip_vrep -> tip position: %f %f %f", position[0], position[1], position[2]);
	#endif // DEBUG

	if (command == 'w') {
		/*	Move tip radially outwards	*/

		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2) + (0.008*duty);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle) - position[0];
		position[1] = hype * sinf(angle) - position[1];
		#ifdef DEBUG
				printf("begining control sequence\n");
		#endif // DEBUG
		/*	Move the arm to the desired location	*/
		control_kinematics(info_ptr, move_ptr, position[0], position[1], 0);

	}
	else if (command == 's') {
		simxFloat hype_2 = (simxFloat)(position[0] * position[0] + position[1] * position[1]);
		simxFloat hype = sqrtf(hype_2) - (0.008*duty);
		simxFloat angle = atan2f(position[1], position[0]);
		/*	set new position	*/
		position[0] = hype * cosf(angle) - position[0];
		position[1] = hype * sinf(angle) - position[1];

		#ifdef DEBUG
			printf("begining control sequence\n");
		#endif // DEBUG
		control_kinematics(info_ptr, move_ptr, position[0], position[1], 0);

	}
	else if (command == 'd') {
		#ifdef DEBUG
			printf("Pivoting +\n");
		#endif // DEBUG
	
		move_joint_angle_vrep(info_ptr, move_ptr, 1, 4.0*duty, false);

	}
	else if (command == 'a') {
		#ifdef DEBUG
			printf("Pivoting -\n");
		#endif // DEBUG
					
		move_joint_angle_vrep(info_ptr, move_ptr, 1, -4.0*duty, false);

	}
	else if (command == '+') {
		
		#ifdef DEBUG
			printf("begining control sequence\n");
		#endif // DEBUG
		control_kinematics(info_ptr, move_ptr, 0, 0, 0.008*duty);

	}
	else if (command == '-') {
		
		#ifdef DEBUG
			printf("begining control sequence\n");
		#endif // DEBUG
		control_kinematics(info_ptr, move_ptr, 0, 0, -0.008*duty);

	}

}



void pause_communication_vrep(info* info_ptr, int status) {

	if (status > 0) {
		simxPauseCommunication(info_ptr->clientID, 1);
	}
	else {
		simxPauseCommunication(info_ptr->clientID, 0);
	}

}



double current_angle(move* move_ptr, int jointNum) {

	double returnAngle = 0.0;


#ifdef DATA_STREAMING

	/*	Received new value, toggle mutex to enter value	*/
	DWORD bufferWaitResult = WaitForSingleObject(jointMutexes[jointNum], INFINITE);  // no time-out interval

	if (bufferWaitResult == WAIT_ABANDONED) {
		printf("Mutex in indeterminate state");
	}
	else if (bufferWaitResult == WAIT_OBJECT_0) {

		returnAngle = move_ptr->currAng[jointNum];

		/*	Release Mutex for this joint	*/
		if (!ReleaseMutex(jointMutexes[jointNum]))
		{
			// Handle error.
			printf("Could not release Mutex %d\n", GetCurrentThreadId());
		}
	}

	
#else


	returnAngle = move_ptr->currAng[jointNum];

	
#endif // !DATA_STREAMING

	return returnAngle;

}