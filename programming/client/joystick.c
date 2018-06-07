/*
	METR4901		2018		UQ
	Callum Rohweder
	joystick.c

	For handling communication with the Joystick process,
	including it's creation, and checking that a joystick
	is connected
*/


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <strsafe.h>
#include <windows.h> 
#include <tchar.h>
#include <string.h>
#include <math.h>
#include "project.h"
#include <time.h>
#include <sys/types.h>
#include <direct.h>

/*	Defines for buffer sizes	*/
#define BUFSIZE 4096					// size of buffer for reading
#define INPUT_BUFFER_SIZE 16			// size of buffer for input bytes
#define OUTPUT_BUFFER_SIZE 255			// size of out buffer

/*	Handles to mutexes and child process	*/
HANDLE  bufferMutex;
HANDLE g_hChildStd_IN_Rd = NULL;
HANDLE g_hChildStd_IN_Wr = NULL;
HANDLE g_hChildStd_OUT_Rd = NULL;
HANDLE g_hChildStd_OUT_Wr = NULL;
HANDLE g_hInputFile = NULL;

/*	Global variables - descriptions provided below	*/
volatile uint8_t out_insert_pos;
volatile uint8_t bytes_in_out_buffer;
volatile char input_buffer_cmd[INPUT_BUFFER_SIZE];		// to store a character representing the button pressed
volatile int input_buffer_val[INPUT_BUFFER_SIZE];		// to store the value of the button pressed
volatile uint8_t input_insert_pos;
volatile uint8_t bytes_in_input_buffer;
volatile uint8_t input_overrun;
volatile long ltime;
volatile long delay;
volatile uint8_t joystickToggle;
volatile int lastJoystickValue;
volatile char lastJoystickCmd;
volatile uint8_t joystickCheck;
volatile uint8_t joystickEn;
volatile uint8_t access;


/*	Function declarations	*/
void interpret_joystick(char* buffer);
void CreateChildProcess(void);
int joystick_input_available(void);
void joystick_clear_input_buffer(void);
void joystick_get_char(info* info_ptr);
void add_to_buffer(void);
uint8_t joystickEnabled(void);



/*	Defines the cmd buffer mutex	*/
extern void initialise_joystick_mutex(void) {
	
	bufferMutex = CreateMutex(NULL, FALSE, NULL);

	if (bufferMutex == NULL)
	{
		printf("CreateMutex error: %d\n", GetLastError());
	}
}


/*
	Initialises the buffers used for storing joystick commands,
	as well as information for getting whether or not a joystick
	is connected. The joystick process is then created, this function
	was mainly an example from MSDN.
*/
void CreateChildProcess() {
	
	access = 0xFF;				// depricated, allowed access to variables before mutex's were discovered
	out_insert_pos = 0;			// contains the insert position for added cmds to out buffer
	bytes_in_out_buffer = 0;	// contains the number of commands being outputted to the child process
	input_insert_pos = 0;		// contains the insert position for new commands being added to the buffer
	bytes_in_input_buffer = 0;	// contains the number of cmds currently in the buffer
	input_overrun = 0;			// represents the overflow of the cmd buffer
	joystickToggle = 0x00;		// represents a new joystick movement
	ltime = clock();			// returns ms as a long
	delay = 200;				// 500 ms delay between writing to buffer
	lastJoystickCmd = '\0';		// set the last command to 0
	lastJoystickValue = 0;		// and the last value to 0
	joystickCheck = 0x00;		// the joystick has not been checked yet
	joystickEn = 0xFF;			// but until proven otherwise, it is assumed to be there

	// Create a child process that uses the previously created pipes for STDIN and STDOUT.
	SECURITY_ATTRIBUTES saAttr;

	// Set the bInheritHandle flag so pipe handles are inherited. 

	saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
	saAttr.bInheritHandle = TRUE;
	saAttr.lpSecurityDescriptor = NULL;

	// Create a pipe for the child process's STDOUT. 

	if (!CreatePipe(&g_hChildStd_OUT_Rd, &g_hChildStd_OUT_Wr, &saAttr, 0)) {
		printf("StdoutRd CreatePipe");
		exit(3);
	}
	// Ensure the read handle to the pipe for STDOUT is not inherited.

	if (!SetHandleInformation(g_hChildStd_OUT_Rd, HANDLE_FLAG_INHERIT, 0)) {
		printf("Stdout SetHandleInformation"); exit(4);

		// Create a pipe for the child process's STDIN. 
	}
	if (!CreatePipe(&g_hChildStd_IN_Rd, &g_hChildStd_IN_Wr, &saAttr, 0)) {
		printf("Stdin CreatePipe");
		exit(5);

		// Ensure the write handle to the pipe for STDIN is not inherited. 
	}
	if (!SetHandleInformation(g_hChildStd_IN_Wr, HANDLE_FLAG_INHERIT, 0)) {
		printf("Stdin SetHandleInformation");
		exit(6);
	}

	
	char* cwd = _getcwd(NULL, 0);			// get the current working directory

	/*	construct the file path	*/
	char* joystickPath = malloc(sizeof(char)*(strlen(cwd) + strlen("/programming/SDL2-2.0.7/VisualC/Win32/Debug/testjoystick") + 5));
	strcpy(joystickPath, cwd);
	strcat(joystickPath, "\\..\\..\\..\\");
	strcat(joystickPath, "SDL2-2.0.7\\VisualC\\Win32\\Debug\\testjoystick");

	#ifdef JOYSTICK_DEBUG
		printf("\n%s\n\n", joystickPath);
	#endif // JOYSTICK_DEBUG

	
	TCHAR szCmdline[] = TEXT("C:/Users/Callum/Documents/2017/METR4901/programming/SDL2-2.0.7/VisualC/Win32/Debug/testjoystick");
	//TCHAR szCmdline[] = joystickPath;//TEXT(joystickPath);
	/*	Spawn Process	*/
	TCHAR szCurrentDirectory[] = TEXT("");
	PROCESS_INFORMATION piProcInfo;
	STARTUPINFO siStartInfo;
	BOOL bSuccess = FALSE;

	// Set up members of the PROCESS_INFORMATION structure. 

	ZeroMemory(&piProcInfo, sizeof(PROCESS_INFORMATION));

	// Set up members of the STARTUPINFO structure. 
	// This structure specifies the STDIN and STDOUT handles for redirection.

	ZeroMemory(&siStartInfo, sizeof(STARTUPINFO));
	siStartInfo.cb = sizeof(STARTUPINFO);
	siStartInfo.hStdError = g_hChildStd_OUT_Wr;
	siStartInfo.hStdOutput = g_hChildStd_OUT_Wr;
	siStartInfo.hStdInput = g_hChildStd_IN_Rd;
	siStartInfo.dwFlags |= STARTF_USESTDHANDLES;

	// Create the child process. 

	bSuccess = CreateProcess(NULL,
		joystickPath,	// command line 
		NULL,          // process security attributes 
		NULL,          // primary thread security attributes 
		TRUE,          // handles are inherited 
		0,             // creation flags 
		NULL,          // use parent's environment 
		NULL,          // use parent's current directory 
		&siStartInfo,  // STARTUPINFO pointer 
		&piProcInfo);  // receives PROCESS_INFORMATION 

					   // If an error occurs, exit the application. 
	if (!bSuccess) {
		printf("Creating Process not a success");
			printf("%s\n", joystickPath); fflush(stdout); exit(8);
	}
	else {
		// Close handles to the child process and its primary thread.
		// Some applications might keep these handles to monitor the status
		// of the child process, for example. 

		CloseHandle(piProcInfo.hProcess);
		CloseHandle(piProcInfo.hThread);
	}
}


/*	Returns true when there are bytes in the buffer to be
	interpreted by the main thread and acted on	*/
int joystick_input_available(void) {

	return (bytes_in_input_buffer != 0);
}


/* Clears the input command buffer by
	changing the insert position and number
	of bytes in the buffer so it looks empty */
void joystick_clear_input_buffer(void) {
	
	input_insert_pos = 0;
	bytes_in_input_buffer = 0;
}


/*	Gets a cmd from the joystick input buffer. The command
	is a movement, w/a/s/d, with a corresponding amount of ON.
	Toggling on a joystick axis will return a signed 16-bit int,
	this is contained in input_buffer_val, and the movement in
	input_buffer_cmd. These will be extracted and placed into
	info_ptr->response, so it can be processed as if it
	was from a keyboard input.	*/
void joystick_get_char(info* info_ptr) {
	
	while (bytes_in_input_buffer == 0) {
		/* do nothing until a character is received */

	}
	int i = 0;
	char c = 'l';

	DWORD bufferWaitResult = WaitForSingleObject(bufferMutex, INFINITE);  // no time-out interval

	if (bufferWaitResult == WAIT_ABANDONED) {
		printf("Mutex in indeterminate state");
	}
	else if (bufferWaitResult == WAIT_OBJECT_0) {

		/*	check for buffer overflow	*/
		if (input_insert_pos - bytes_in_input_buffer < 0) {
			
			/* Need to wrap around */
			#ifdef JOYSTICK_DEBUG
				printf(" / %c %c / ", input_buffer_cmd[0], input_buffer_cmd[input_insert_pos - bytes_in_input_buffer + INPUT_BUFFER_SIZE]);
			#endif

			c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer
				+ INPUT_BUFFER_SIZE];
			i = input_buffer_val[input_insert_pos - bytes_in_input_buffer
				+ INPUT_BUFFER_SIZE];
		}
		/*	the buffer didn't overflow, use the index determined by number
			of bytes in the buffer and the inserting position	*/
		else {
			
			#ifdef JOYSTICK_DEBUG
				printf(" ? %c %c ? ", input_buffer_cmd[0], input_buffer_cmd[input_insert_pos - bytes_in_input_buffer]);
			#endif
			
			c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer];
			i = input_buffer_val[input_insert_pos - bytes_in_input_buffer];
		}

		/* Decrement our count of bytes in the input buffer */
		bytes_in_input_buffer--;

		access = 0xFF;
		#ifdef DEBUG
			printf("joystick.c: %c %d\n", c, i);
		#endif // DEBUG


		info_ptr->response = malloc(sizeof(char) * 128);
		/*	the input is converted into the same format that command line ik
		*	inputs come in to allow joystick or command line inputs	*/

		sprintf(info_ptr->response, "%c %d", c, i);

		printf("received: %s\n", info_ptr->response);

		if (!ReleaseMutex(bufferMutex))
		{
			// Handle error.
			printf("Could not release Mutex %d\n", GetCurrentThreadId());
		}
	}
}


/* Depreciated function for writing to STDIN of process */ 
void WriteToPipe(void) {
	

	printf("in WriteToPipe\n");
	DWORD dwRead, dwWritten;
	CHAR chBuf[BUFSIZE];
	BOOL bSuccess = FALSE;


	bSuccess = ReadFile(g_hInputFile, chBuf, BUFSIZE, &dwRead, NULL);
	if (!bSuccess || dwRead == 0) {
		if (!bSuccess) {
			printf("failed reading from file\n"); //break; 
		}
		printf("dwRead == 0? %d\n %s", dwRead, chBuf);
		
	}

	bSuccess = WriteFile(g_hChildStd_IN_Wr, chBuf, dwRead, &dwWritten, NULL);
	if (!bSuccess) {
		printf("failed writing to child\n"); //break; 
	}


	// Close the pipe handle so the child process stops reading. 

	if (!CloseHandle(g_hChildStd_IN_Wr)) {
		printf("StdInWr CloseHandle"); exit(9);
	}
	else { printf("wrote some thusgas\n"); }
	fflush(stdout);
}


/*	Read from STDOUT of the joystick process.
	When data has been received, break it into 
	lines for interpretting. The incomming commands
	will describe the status of the joystick (connected
	or disconnected), and the status of buttons and axis 
	(of the two joysticks/toggle sticks of the gaming controller).
	This thread will exit when there isnt a controller connected.	*/
void ReadFromPipe() {

	DWORD dwRead, dwWritten;
	CHAR chBuf[BUFSIZE];
	BOOL bSuccess = FALSE;
	HANDLE hParentStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	
	// prompt that the input type (joystick or keyboard) is being
	// decided by the information that is returned by the process
	printf("Input Type: "); fflush(stdout);
	
	/*	forever check for inputs from the joystick process	*/
	for (;;)
	{
		bSuccess = ReadFile(g_hChildStd_OUT_Rd, chBuf, BUFSIZE, &dwRead, NULL);
		
		// ensure there wasn't an error in reading
		if (!bSuccess || dwRead == 0) { 
			printf("failed reading from child\n"); 
			break; 
		}

		// ensure that the buffer is full
		if (chBuf[0] != 0) {

			// the buffer contained information, extract new-lines to a buffer that can
			// be manipulated
			char* fullBuffer = malloc(sizeof(char)*(strlen(chBuf) + 1));
			strcpy(fullBuffer, chBuf);
			char* line = strtok(fullBuffer, "\n");
			
			// until there isn't a new line to be interpreted
			while (line != NULL) {

				// copy the line to a buffer that can be interpreted
				char* buffer = malloc(sizeof(char)*(strlen(line) + 1));
				strcpy(buffer, line);
				
				#ifdef JOYSTICK_DEBUG
					printf("Buffer going in:	#%s#\n", buffer); fflush(stdout);
				#endif //JOYSTICK_DEBUG
				
				// handle interpretting the buffer
				interpret_joystick(buffer);

				// free the buffer so it can be created again next time information comes in
				free(buffer);

				// get the next line from the message received
				line = strtok(NULL, "\n");
			}
			free(fullBuffer);
			
		}

		// check if there was an error in communication
		if (!bSuccess) { printf("failed reading from stdin\n"); break; }

		// reset the buffer for next iteration
		memset(chBuf, 0, BUFSIZE);

	}
}


/*	Filters through a line given by the joystick process.
	The line will provide information on button presses,
	axis changes, or whether there is a joystick connected	*/
void interpret_joystick(char* buffer) {

	int wordCount = 1;
	int isButton = 0;
	int isAxis = 0;

	/*	Check if we have checked that a joystick is attached	*/
	if (!joystickCheck){

		/*	Check for "No Joystick" in line	*/
		if ((buffer[0] == 'N') && (buffer[1] == 'o') && (buffer[3] == 'J')){

			/*	Joystick is not plugged in, look to command prompt for inputs	*/
			joystickEn = 0x00;				// there is not a joystick attached
			joystickCheck = 0xFF;			// joystick check is true
			printf("Keyboard\n"); fflush(stdout);
			ExitThread(1);					// this thread should exit as there is not a joystick to read
		}
		else {

			/*	Joystick is enabled and running as expected	*/
			joystickCheck = 0xFF;			// joystick check is true
			joystickEn = 0xFF;				// joystick flag for connected set to high
			printf("Joystick\n"); fflush(stdout);
			
		}
		
		return;
	}

	/*	Check the buffer for a button press or axis command	*/
	for (int i = 0; i < (strlen(buffer)); i++) {

		if (buffer[i] == ' ') {
			++wordCount;
		}
		else if ((buffer[i] == 'v') && (buffer[i+1] == 'a') && (buffer[i + 2] == 'l')) {
			
			#ifdef JOYSTICK_DEBUG
				printf("isAxis: %s\n", buffer);
			#endif //JOYSTICK_DEBUG
			++isAxis;
		}
		else if ((buffer[i] == 'b') && (buffer[i + 1] == 'u') && (buffer[i + 2] == 't')) {

			#ifdef JOYSTICK_DEBUG
						printf("isbutton: %s\n", buffer);
			#endif //JOYSTICK_DEBUG

			++isButton;
		}
	}

	/*	check that a button press of axis status is given	*/
	if ((isButton < 1) && (isAxis < 1)) {
		// then something is received that isn't of interest
		return;
	}

		
	int actuator;
	char* token;
	int value;
	token = strtok(buffer, " ");
	
	#ifdef JOYSTICK_DEBUG
		printf("token1: %s\n", token); fflush(stdout);
	#endif //JOYSTICK_DEBUG
	
	for (int words = 1; words < 7; words++) {
		
		if (token == NULL) {
			// there isnt any more words to interpret
			break;
		}
		
		#ifdef JOYSTICK_DEBUG
				printf("token %d: %s\n", words, token); fflush(stdout);
		#endif //JOYSTICK_DEBUG
		
		if (words == 4) {

			char* err;
			actuator = strtol(token, &err, 10);

		/*	check if it was an axis change	*/
		} else if ((isAxis > 0) && (words == 6)) {

			char* err;
			value= strtol(token, &err, 10);

		}
		/*	check if it was a button press	*/
		else if ((isButton > 0) && (words == 5)) {

			/*	check if the button was up or down	*/
			if (strcmp(token, "up") > 0) {
				value = 0;
			}
			else {
				value = 1;
			}
		}
		token = strtok(NULL, " ");
	}

	if (value == 0) {
		joystickToggle = 0x00;
		
		#ifdef JOYSTICK_DEBUG
				printf("joysticktoggle OFF\n");
		#endif //JOYSTICK_DEBUG
		
		return;
	}
	else if (value != lastJoystickValue) {
		
		#ifdef JOYSTICK_DEBUG
				printf("joysticktoggle ON\n");
		#endif //JOYSTICK_DEBUG
		
		joystickToggle = 0xFF;
	}

	#ifdef JOYSTICK_DEBUG
		printf("Button: %d,	Axis: %d,	actuator: %d	value:	%d\n", isButton, isAxis, actuator, value); fflush(stdout);
	#endif //JOYSTICK_DEBUG
	
	char cmd;
	if (isButton > 0) {
		switch (actuator) {
			case 0:
				cmd = '1';
				break;
			case 1:
				cmd = '2';
				break;
			case 2:
				cmd = '3';
				break;
			case 3:
				cmd = '4';
				break;
			case 4:
				cmd = '5';
				break;
			case 5:
				cmd = '6';
				break;
			case 6:
				cmd = '7';
				break;
			case 7:
				cmd = '8';
				break;
		}
	}
	else if (isAxis > 0) {
		switch (actuator) {
			case 0:
				if (value < 0) {
					cmd = 'a';
					value = value * (-1);
				}
				else {
					cmd = 'd';
				}
				break;
			case 1:
				if (value < 0) {
					cmd = 'w';
					value = value * (-1);
				}
				else {
					cmd = 's';
				}
				break;
			case 2:
				return;
				break;
			case 4:
				if (value < 0) {
					cmd = '+';
					value = value * (-1);
				}
				else {
					cmd = '-';
				}
				break;
		}
	}

	/*
	* Check if we have space in our buffer. If not, set the overrun
	* flag and throw away the character. (We never clear the
	* overrun flag - it's up to the programmer to check/clear
	* this flag if desired.)
	*/
	if (bytes_in_input_buffer >= INPUT_BUFFER_SIZE) {
		input_overrun = 1;
	}
	else {

		/*
		* There is room in the input buffer
		*/
		if ((delay + ltime < clock()) || (value > 30000)) {

			/*	lock the buffer from other threads	*/

			DWORD bufferWaitResult = WaitForSingleObject(bufferMutex, INFINITE);  // no time-out interval

			if (bufferWaitResult == WAIT_ABANDONED) {
				printf("Mutex in indeterminate state\n");
			}
			else if (bufferWaitResult == WAIT_OBJECT_0) {


				input_buffer_cmd[input_insert_pos] = cmd;

				#ifdef DEBUG
					printf("input buff: %c %d\n", input_buffer_cmd[input_insert_pos], value);
				#endif // DEBUG

				input_buffer_val[input_insert_pos] = value;
				lastJoystickCmd = cmd;
				lastJoystickValue = value;
				input_insert_pos++;
				bytes_in_input_buffer++;

				if (input_insert_pos == INPUT_BUFFER_SIZE) {
					/* Wrap around buffer pointer if necessary */
					input_insert_pos = 0;
				}
				/*	Release the buffer and update the delay clock	*/
				//access = 0xFF;
				ltime = clock();

				if (!ReleaseMutex(bufferMutex))
				{
					// Handle error.
					printf("Could not release Mutex %d\n", GetCurrentThreadId());
				}
			}
		}
	}

	Sleep(10);

}


/*	adds commands to the buffer if the joystick is held
	and a given amount of time as passed since updating	*/
void add_to_buffer(void) {
	
	/*	Forever check if things need to be added to the buffer	*/
	for (;;) {

		/*	Check that time has passed, the joystick has been checked and is plugged in,
			and whether the last command needs to be added	*/
		if ((delay + 750 + ltime < clock()) && (joystickToggle) && (joystickCheck)) {

			/*	lock the buffer variables from other threads	*/
			DWORD bufferWaitResult = WaitForSingleObject(bufferMutex, INFINITE);  // no time-out interval

			if (bufferWaitResult == WAIT_ABANDONED) {
				printf("Mutex in indeterminate state\n");
			}
			else if (bufferWaitResult == WAIT_OBJECT_0) {


				/*	ensure the commands are valid before adding them to the cmd buffer	*/
				if ((lastJoystickCmd > 'a') && (lastJoystickCmd < 'z')) {

					#ifdef DEBUG
						printf("thread %d adding extra %c %d to buffer\n", GetCurrentThreadId(), lastJoystickCmd, lastJoystickValue);
					#endif // DEBUG

					/*	add the last added command to the buffer	*/
					input_buffer_cmd[input_insert_pos] = lastJoystickCmd;
					input_buffer_val[input_insert_pos] = lastJoystickValue;

					/*	update position in bufffer	*/
					input_insert_pos++;
					bytes_in_input_buffer++;

					if (input_insert_pos == INPUT_BUFFER_SIZE) {
						/* Wrap around buffer pointer if necessary */
						input_insert_pos = 0;
					}

				}

				/*	Release the buffer and update delay clock	*/
				ltime = clock();

				if (!ReleaseMutex(bufferMutex))
				{
					// Handle error.
					printf("Could not release mutex %d\n", GetCurrentThreadId());
				}

				

			}
		}

		/*	Check that the joystick has been checked and it is not connected	*/
		if ((joystickCheck) && (!joystickEn)) {

			// then the thread needs to exit as it is not needed
			ExitThread(1);
		}

		// block this thread before checking if need to add command again
		Sleep(50);

	}
	
}



/*	Returns whether there is a joystick plugged in
	and ready for use	*/
uint8_t joystickEnabled(void) {
	
	return joystickEn;
}