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

#define BUFSIZE 4096 

HANDLE g_hChildStd_IN_Rd = NULL;
HANDLE g_hChildStd_IN_Wr = NULL;
HANDLE g_hChildStd_OUT_Rd = NULL;
HANDLE g_hChildStd_OUT_Wr = NULL;

HANDLE g_hInputFile = NULL;

#define OUTPUT_BUFFER_SIZE 255
volatile uint8_t out_insert_pos;
volatile uint8_t bytes_in_out_buffer;

#define INPUT_BUFFER_SIZE 16
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
void interpret_joystick(char* buffer);
void CreateChildProcess(void);
int joystick_input_available(void);
void joystick_clear_input_buffer(void);
void joystick_get_char(info* info_ptr);
void add_to_buffer(void);
uint8_t joystickEnabled(void);

volatile uint8_t access;

void CreateChildProcess() {
	/*
	* Initialise our buffers
	*/
	access = 0xFF;
	out_insert_pos = 0;
	bytes_in_out_buffer = 0;
	input_insert_pos = 0;
	bytes_in_input_buffer = 0;
	input_overrun = 0;
	joystickToggle = 0x00;
	ltime = clock();			// returns ms as a long
	delay = 200;				// 500 ms delay between writing to buffer
	lastJoystickCmd = '\0';
	lastJoystickValue = 0;
	joystickCheck = 0x00;
	joystickEn = 0xFF;
	// Create a child process that uses the previously created pipes for STDIN and STDOUT.
	SECURITY_ATTRIBUTES saAttr;

	//printf("\n->Start of parent execution.\n");

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

	/*	Spawn Process	*/
	TCHAR szCmdline[] = TEXT("C:/Users/Callum/Documents/2017/METR4901/programming/SDL2-2.0.7/VisualC/Win32/Debug/testjoystick");
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
		szCmdline,     // command line 
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
		printf("Creating Process not a success"); fflush(stdout); exit(8);
	}
	else {
		// Close handles to the child process and its primary thread.
		// Some applications might keep these handles to monitor the status
		// of the child process, for example. 

		CloseHandle(piProcInfo.hProcess);
		CloseHandle(piProcInfo.hThread);
	}
}


int joystick_input_available(void) {
	//printf("bytes: %d\n", bytes_in_input_buffer); fflush(stdout);
	return (bytes_in_input_buffer != 0);
}


void joystick_clear_input_buffer(void) {
	/* Just adjust our buffer data so it looks empty */
	input_insert_pos = 0;
	bytes_in_input_buffer = 0;
}


void joystick_get_char(info* info_ptr) {
	/* Wait until we've received a character */
	while (bytes_in_input_buffer == 0) {
		/* do nothing */
		//printf(",,");
	}
	int i = 0;
	char c = 'l';

	while (!access) {
		;
	}
	access = 0x00;

	if (input_insert_pos - bytes_in_input_buffer < 0) {
		/* Need to wrap around */
		//printf(" / %c %c / ", input_buffer_cmd[0], input_buffer_cmd[input_insert_pos - bytes_in_input_buffer + INPUT_BUFFER_SIZE]);
		c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer
			+ INPUT_BUFFER_SIZE];
		i = input_buffer_val[input_insert_pos - bytes_in_input_buffer
			+ INPUT_BUFFER_SIZE];
	}
	else {
		//printf(" ? %c %c ? ", input_buffer_cmd[0], input_buffer_cmd[input_insert_pos - bytes_in_input_buffer]);
		c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer];
		i = input_buffer_val[input_insert_pos - bytes_in_input_buffer];
	}

	/* Decrement our count of bytes in the input buffer */
	bytes_in_input_buffer--;
	
	access = 0xFF;

	printf("joystick.c: %c %d\n", c, i);									// this shows nothing for c
	
																			
	//int* arr = malloc(sizeof(int) * 2);
	//joystick_get_char(arr);


	info_ptr->response = malloc(sizeof(char) * 128);
	/*	the input is converted into the same format that command line ik
	*	inputs come in to allow joystick or command line inputs	*/

	sprintf(info_ptr->response, "%c %d", c, i);

	printf("received: %s\n", info_ptr->response);

	//free(arr);
																			
																			
																			//int arr[2] = { c, i };													// prints c as 0
	
	//arr[0] = c;
	//arr[1] = i;
	
	//return arr;
}



void WriteToPipe(void) {

	// Read from a file and write its contents to the pipe for the child's STDIN.
	// Stop when there is no more data. 

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


void ReadFromPipe() {

	// Read output from the child process's pipe for STDOUT
	// and write to the parent process's pipe for STDOUT. 
	// Stop when there is no more data. 

	DWORD dwRead, dwWritten;
	CHAR chBuf[BUFSIZE];
	BOOL bSuccess = FALSE;
	HANDLE hParentStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	printf("\nInput Type: "); fflush(stdout);
	//printf("we made it here\n"); fflush(stdout);
	for (;;)
	{
		bSuccess = ReadFile(g_hChildStd_OUT_Rd, chBuf, BUFSIZE, &dwRead, NULL);
		if (!bSuccess || dwRead == 0) { printf("failed reading from child\n"); break; }

		//bSuccess = WriteFile(hParentStdOut, chBuf,dwRead, &dwWritten, NULL);

		//printf("%ld %ld\n", delay + ltime, clock());
		if (chBuf[0] != 0) {
			char* fullBuffer = malloc(sizeof(char)*(strlen(chBuf) + 1));
			strcpy(fullBuffer, chBuf);
			char* line = strtok(fullBuffer, "\n");
			while (line != NULL) {

				char* buffer = malloc(sizeof(char)*(strlen(line) + 1));
				strcpy(buffer, line);
				//fflush(stdout);
				//printf("Buffer going in:	#%s#\n", buffer); fflush(stdout);
				//buffer[strlen(line)] = '\0';
				interpret_joystick(buffer);
				free(buffer);

				line = strtok(NULL, "\n");
			}
			free(fullBuffer);
			
		}
		if (!bSuccess) { printf("failed writing to stdout\n"); break; }
		memset(chBuf, 0, BUFSIZE);
	}
}


void interpret_joystick(char* buffer) {

	int wordCount = 1;
	int isButton = 0;
	int isAxis = 0;

	if (!joystickCheck){

		if ((buffer[0] == 'N') && (buffer[1] == 'o') && (buffer[3] == 'J')){
			/*	Joystick is not plugged in, look to command prompt for inputs	*/
			joystickEn = 0x00;
			printf("Keyboard\n"); fflush(stdout);
		}
		else {
			/*	Joystick is enabled and running as expected	*/
			joystickEn = 0xFF;
			printf("Joystick\n"); fflush(stdout);
			
		}
		joystickCheck = 0xFF;
		return;
	}

	for (int i = 0; i < (strlen(buffer)); i++) {

		if (buffer[i] == ' ') {
			++wordCount;
		}
		else if ((buffer[i] == 'v') && (buffer[i+1] == 'a') && (buffer[i + 2] == 'l')) {
			//printf("isAxis: %s\n", buffer);
			++isAxis;
		}
		else if ((buffer[i] == 'b') && (buffer[i + 1] == 'u') && (buffer[i + 2] == 't')) {
			//printf("isbutton: %s\n", buffer);
			++isButton;
		}
	}

	if ((isButton < 1) && (isAxis < 1)) {
		//printf("returning\n");
		return;
	}


	//printf("is something\n"); fflush(stdout);
		
	int actuator;
	char* token;
	int value;
	token = strtok(buffer, " ");
	//printf("token1: %s\n", token); fflush(stdout);

	for (int words = 1; words < 7; words++) {
		if (token == NULL) {
			break;
		}
		
		//printf("token %d: %s\n", words, token); fflush(stdout);
		if (words == 4) {
			//printf("words=3 %s\n", token); fflush(stdout);
			char* err;
			actuator = strtol(token, &err, 10);
			//printf("This is actuator: %s %d\n", token, actuator);
		} else if ((isAxis > 0) && (words == 6)) {
			//printf("words=5 %s\n", token); fflush(stdout);
			char* err;
			value= strtol(token, &err, 10);
			//printf("This is actuator: %s %d\n", token, value);
		}
		else if ((isButton > 0) && (words == 5)) {
			//printf("############\n");
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
		//printf("joysticktoggle OFF\n");
		return;
	}
	else if (value != lastJoystickValue) {
		//printf("joysticktoggle ON\n");
		joystickToggle = 0xFF;
	}

	//printf("Button: %d,	Axis: %d,	actuator: %d	value:	%d\n", isButton, isAxis, actuator, value); fflush(stdout);
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
			while (!access) {
				;
			}
			access = 0x00;

			input_buffer_cmd[input_insert_pos] = cmd;
			printf("input buff: %c %d\n", input_buffer_cmd[input_insert_pos], value);
			input_buffer_val[input_insert_pos] = value;
			lastJoystickCmd = cmd;
			lastJoystickValue = value;
			input_insert_pos++;
			bytes_in_input_buffer++;
			
			if (input_insert_pos == INPUT_BUFFER_SIZE) {
				/* Wrap around buffer pointer if necessary */
				input_insert_pos = 0;
			}
			
			access = 0xFF;
			ltime = clock();
		}
	}
}


void add_to_buffer(void) {
	/*	adds stuff to the buffer if the joystick is held	*/	
	while (1) {
		if ((delay + 750 + ltime < clock()) && (joystickToggle) && (joystickCheck)) {
			while (!access) {
				;
			}
			access = 0x00;
			printf("adding extra %c %d to buffer\n", lastJoystickCmd, lastJoystickValue);
			input_buffer_cmd[input_insert_pos] = lastJoystickCmd;
			input_buffer_val[input_insert_pos] = lastJoystickValue;
			input_insert_pos++;
			bytes_in_input_buffer++;
			if (input_insert_pos == INPUT_BUFFER_SIZE) {
				/* Wrap around buffer pointer if necessary */
				input_insert_pos = 0;
			}
			access = 0xFF;
			ltime = clock();
		}
	}
	
}




uint8_t joystickEnabled(void) {
	/*	Returns whether there is a joystick plugged in
	 *	and ready for use	*/

	return joystickEn;
}