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

void interpret_joystick(char* buffer);
void CreateChildProcess(void);
int joystick_input_available(void);
void joystick_clear_input_buffer(void);
int* joystick_get_char(void);


void CreateChildProcess() {
	/*
	* Initialise our buffers
	*/
	out_insert_pos = 0;
	bytes_in_out_buffer = 0;
	input_insert_pos = 0;
	bytes_in_input_buffer = 0;
	input_overrun = 0;

	// Create a child process that uses the previously created pipes for STDIN and STDOUT.
	SECURITY_ATTRIBUTES saAttr;

	printf("\n->Start of parent execution.\n");

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
		printf("Creating Process not a success"); exit(8);
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
	return (bytes_in_input_buffer != 0);
}


void joystick_clear_input_buffer(void) {
	/* Just adjust our buffer data so it looks empty */
	input_insert_pos = 0;
	bytes_in_input_buffer = 0;
}


int* joystick_get_char(void) {
	/* Wait until we've received a character */
	while (bytes_in_input_buffer == 0) {
		/* do nothing */
	}
	int i;
	char c;
	if (input_insert_pos - bytes_in_input_buffer < 0) {
		/* Need to wrap around */

		c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer
			+ INPUT_BUFFER_SIZE];
		i = input_buffer_val[input_insert_pos - bytes_in_input_buffer
			+ INPUT_BUFFER_SIZE];
	}
	else {

		c = input_buffer_cmd[input_insert_pos - bytes_in_input_buffer];
		i = input_buffer_val[input_insert_pos - bytes_in_input_buffer];
	}

	/* Decrement our count of bytes in the input buffer */
	bytes_in_input_buffer--;
	int arr[2] = { c, i };
	return arr;
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
		//break;
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
	//printf("we made it here\n"); fflush(stdout);
	for (;;)
	{
		bSuccess = ReadFile(g_hChildStd_OUT_Rd, chBuf, BUFSIZE, &dwRead, NULL);
		if (!bSuccess || dwRead == 0) { printf("failed reading from child\n"); break; }

		//bSuccess = WriteFile(hParentStdOut, chBuf,dwRead, &dwWritten, NULL);

		if (chBuf[0] != 0) {
			char* fullBuffer = malloc(sizeof(char)*(strlen(chBuf) + 1));
			strcpy(fullBuffer, chBuf);
			char* line = strtok(fullBuffer, "\n");
			while (line != NULL) {

				char* buffer = malloc(sizeof(char)*(strlen(line) + 1));
				strcpy(buffer, line);
				//printf("Buffer going in: %s\n", buffer);
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
					cmd = 'd';
					value = value * (-1);
				}
				else {
					cmd = 'a';
				}
				break;
			case 1:
				if (value < 0) {
					cmd = 's';
					value = value * (-1);
				}
				else {
					cmd = 'w';
				}
			case 2:
				return;
				break;
			case 4:
				if (value < 0) {
					cmd = '-';
					value = value * (-1);
				}
				else {
					cmd = '+';
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
		input_buffer_cmd[input_insert_pos++] = cmd;
		input_buffer_val[input_insert_pos++] = value;
		bytes_in_input_buffer++;
		if (input_insert_pos == INPUT_BUFFER_SIZE) {
			/* Wrap around buffer pointer if necessary */
			input_insert_pos = 0;
		}
	}

}