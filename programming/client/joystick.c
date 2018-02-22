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
volatile char input_buffer[INPUT_BUFFER_SIZE];
volatile uint8_t input_insert_pos;
volatile uint8_t bytes_in_input_buffer;
volatile uint8_t input_overrun;


void CreateChildProcess() {
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
	printf("we made it here\n"); fflush(stdout);
	for (;;)
	{
		bSuccess = ReadFile(g_hChildStd_OUT_Rd, chBuf, BUFSIZE, &dwRead, NULL);
		if (!bSuccess || dwRead == 0) { printf("failed reading from child\n"); break; }

		bSuccess = WriteFile(hParentStdOut, chBuf,
			dwRead, &dwWritten, NULL);
		char* buffer = malloc(sizeof(char)*(strlen(chBuf) + 1));
		strcpy(buffer, chBuf);
		interpret_joystick(buffer);
		free(buffer);
		if (!bSuccess) { printf("failed writing to stdout\n"); break; }
	}
}


void interpret_joystick(char* buffer) {



}