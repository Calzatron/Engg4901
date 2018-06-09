/*
	METR4901		2018		UQ
	Callum Rohweder
	kinematics.c

	The kinematics library for the Jaco arm in VREP.
	Contains necessary calculations for forward
	and inverse kinematics.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <strsafe.h>
#include <windows.h> 
#include <tchar.h>
#include <string.h>
#include "project.h"
#include "time.h"
#include <conio.h>

/*	Function Declarations	*/
move* move_ptr;
void define_classic_parameters(move* move_ptr);
void fk_classic(move* move_ptr, info* info_ptr);
void fk_mod(move* move_ptr);
void ik_RRR_arm(move* move_ptr, char* plane);
int inverse_kinematics(move* move_ptr, info* info_ptr, float* position, double* angles);
void control_kinematics(info* info_ptr, move* move_ptr, float x, float y, float z);
void control_kinematics_v2(info* info_ptr, move* move_ptr, float x, float y, float z);
void control_kinematics_v3(info* info_ptr, move* move_ptr, float x, float y, float z);
double determinant(double matrix[25][25], double size);
void cofactor(double matrix[25][25], double **invs, double size, double det);
void transpose(double matrix[25][25], double **invs, double matrix_cofactor[25][25], double size, double det);
double LU_decomposition(double A[10][10], int size);

/*	Global Variables	*/
int Mode1;					// toggles when mode 1 is ON or OFF
int Mode2;					// toggles when mode 2 is ON or OFF
int Mode3;					// toggles when mode 3 is ON or OFF



/*
*	@brief Sets the which INV K solution to use
*	@param command from input "mode # #"
*	@ret none
*/
void inverse_kinematics_mode_toggle(char* inputCommand) {

	/*	copy the buffer so it can be interpretted	*/
	char buffer[10];
	memset(buffer, 0, 10);
	strcpy(buffer, inputCommand);

	/*	Filter at the spaces for words / values	*/
	int cmdError = 0;
	const char s[2] = {' ', '\0'};
	char* token;
	int num = 0;
	token = strtok(buffer, s);
	int modeNum = 0;
	int modeToggle = 0;

	/*	ensure it started with "mode"	*/
	if (strcmp("mode", token) == 0) {

		/*	get the values corresponding to mode number and boolean	*/
		while ((token != NULL) && (num < 2)) {
			
			token = strtok(NULL, s);

			char bbuf[5];
			memset(bbuf, 0, 5);
			sprintf(bbuf, "%s", token);
			char* err;

			/*	check if the mode number has been determined	*/
			if (!num) {

				/*	get the mode number as an int	*/
				modeNum = strtol(bbuf, &err, 10);
				
				/*	ensure it's a valid mode number	*/
				if (modeNum > 3) {
					cmdError = 1;
				}
				++num;
			}
			/*	check if the value, bool, has been found	*/
			else {
				// get the mode toggle value
				modeToggle = strtol(bbuf, &err, 10);
				if (modeToggle > 1) {
					cmdError = 1;
				}

				++num;
			}

		}

	}
	else {
		// otherwise it was not a mode
		printf("not mode str\n");
		cmdError = 1;
	}

	/*	check that the mode number and bool were determined	*/
	if ((!modeNum) && (!modeToggle)) {
		cmdError = 1;
	}

	/*	check if there was a cmd error and show	*/
	if (cmdError) {
		printf("Mode Usage: mode modeNumber(1,2,3) bool(0,1)\n");
		
		/*	print the current mode	*/
		printf("Active Modes: ");
		if (Mode1) {
			printf(" Mode1 ");
		}
		if (Mode2) {
			printf(" Mode2 ");
		}
		if (Mode3) {
			printf(" Mode3 ");
		}
		printf("\n");

		return;
	}

	/*	Otherwise set the the value of the mode to a the toggle value	*/
	else {
		if (modeNum == 1) {
			Mode1 = modeToggle;
		}
		else if (modeNum == 2) {
			Mode2 = modeToggle;
		}
		else if (modeNum == 3) {
			Mode3 = modeToggle;
		}
	}
}


/*	Stores in memory the forward kinematic
*	parameters of the Jaco arm as per the classic DH solution
*/
void define_classic_parameters(move* move_ptr) {
	

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
	move_ptr->d_i[4] = -(move_ptr->lengthD[3] + move_ptr->lengthD[4] * aa);
	move_ptr->d_i[5] = -(move_ptr->lengthD[4] * aa + move_ptr->lengthD[5] * aa);
	move_ptr->d_i[6] = -(move_ptr->lengthD[6] + move_ptr->lengthD[5] * aa);

}


/*	Computes the position of the end-effector in Cartesian coordinates
	relative to the base of the arm, using the angles of the Jaco arm
	in V-REP
*/
void fk_classic(move* move_ptr, info* info_ptr) {

	/*	store the joint angles	*/
	double q_1 = move_ptr->currAng[1 - 1];
	double q_2 = move_ptr->currAng[2 - 1];
	double q_3 = move_ptr->currAng[3 - 1];
	double q_4 = move_ptr->currAng[4 - 1];
	double q_5 = move_ptr->currAng[5 - 1];
	#ifdef DEBUG
		printf("Angles are:		%f %f %f %f %f\n", q_1, q_2, q_3, q_4, q_5);
	#endif // DEBUG


	double pi = 3.141592;

	/*	Transform angles into DH	*/
	double q1, q2, q3, q4, q5, q6;

	q1 = -pi - q_1;
	q2 = -(pi / 2.0) + (q_2 + pi);
	q3 = (pi / 2.0) + (q_3 + pi);
	q4 = q_4;
	q5 = -pi + q_5;
	q6 = 0;

	/*	compute the translational components of the transformation matrix, given the
		current configuration
	*/
	double T[3];

	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;

	T[0] = (T[0] / 1000.0);
	T[1] = (T[1] / 1000.0);
	T[2] = (T[2] / 1000.0);

#ifdef DEBUG
	printf("Calc position of Tip at:		%f %f %f\n", T[0], T[1], T[2]);
#endif // DEBUG


	float tipPosition[3];
	int handle = info_ptr->targetHandle - 1;
	get_world_position_vrep(info_ptr, tipPosition,  handle);

	printf("Found tip to be at:		%f %f %f\n", tipPosition[0] - info_ptr->armPosition[0], tipPosition[1] - info_ptr->armPosition[1], tipPosition[2]);

}


/*	Calculates the position of the tip using classic DH parameters,
*	and the full transformation matrix, such that
*	ret = T * pt. ret is printed to terminal
*/
void fk_classic_old(move* move_ptr, info* info_ptr) {
	

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

	/* Compute the position	*/
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 4; k++) {
				ret[i][j] += T[i][k] * pt[k][j];
			}

		}
	}
	printf("Forward: %f %f %f\n", ret[0][1], ret[1][0], ret[2][0]);
	float position[3];

	/*	changed to world position, should be of base	*/

	get_world_position_vrep(info_ptr, &position, 33);
	printf("%f %f %f\n", position[0], position[1], position[2]);

	get_world_position_vrep(info_ptr, &position, 30);
	printf("%f %f %f\n", position[0], position[1], position[2]);

	get_world_position_vrep(info_ptr, &position, 27);
	printf("%f %f %f\n", position[0], position[1], position[2]);

	get_world_position_vrep(info_ptr, &position, 24);
	printf("%f %f %f\n", position[0], position[1], position[2]);

	get_world_position_vrep(info_ptr, &position, 21);
	printf("%f %f %f\n", position[0], position[1], position[2]);

	get_world_position_vrep(info_ptr, &position, 18);
	printf("%f %f %f\n", position[0], position[1], position[2]);
}


/*	Uses the position of the fourth joint to calculate angles q_2 and q_3
*	for a fourth joint position given by the position buffer.
*	After, the position of S' is calculated (position of the tip for the given
*	and current angles), and placed in the position buffer to be interpreted once returned
*/
int inverse_kinematics(move* move_ptr, info* info_ptr, float* position, double* angles) {
	
	double pi = 3.141594;

	#ifdef DEBUG
		printf("Tip desired: %f %f %f\n", position[0], position[1], position[2]);
	#endif // DEBUG
			
	double px = (double)((position[0])*1000.0);
	double py = (double)((position[1])*1000.0);
	double pz = (double)(position[2])*1000.0;
	double d1 = 197.13;
	double d2 = 410.0;
	double d3 = 207.3;
	double e2 = 9.8;

	double j = pow(px, 2) + pow((pz - d1), 2) - (d2*d2) - (d3*d3);
	double k = 2.0 * d2 * d3;

	if (fabs(j) > fabs(k)) {
		#ifdef DEBUG
		printf("x,z,i,j:	%f %f %f %f\n", px, pz, j, k);
		#endif // DEBUG

		return 1;
	}

	double q_3 = 1 * acos( j / k );
	#ifdef DEBUG
		printf("i,j,q_3:	%f %f %f\n", j, k, q_3);
	#endif // DEBUG
	double cq_2 = (px*(d2 + d3 * cos(q_3)) + d3 * sin(q_3)*(pz - d1)) / (pow(d2,2) + pow(d3,2) + 2 * d2*d3*cos(q_3));
	double sq_2 = (-px * d3*sin(q_3) + (d2 + d3 * cos(q_3))*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
	double q_2 = (pi / 2) - atan2(sq_2, cq_2);
	#ifdef DEBUG
		printf("c,s,q_2:	%f %f %f\n", cq_2, sq_2, q_2);
	#endif // DEBUG
	if (((current_angle(move_ptr, 3 - 1) - pi / 4 < q_3) && (current_angle(move_ptr, 3 - 1) + pi / 4 > q_3)) ||
				((q_2 < 0) && (q_3 > 0)) || ((q_2 > 0) && (q_3 < 0))){
		/*	the new q_3 is within reach of the current joint 3's position	*/
	#ifdef DEBUG
		printf("check ");
	#endif // DEBUG
				
	}
	else {
		/*	the wrong angle reflection was chosen, re-calculate	*/

		q_3 = -1 * q_3;
		cq_2 = (px*(d2 + d3 * cos(q_3)) + d3 * sin(q_3)*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
		sq_2 = (-px * d3*sin(q_3) + (d2 + d3 * cos(q_3))*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
		q_2 = (pi / 2) - atan2f(sq_2, cq_2);
	}

	// Instead of calculating q_1 should read from vrep
	double q_1 = current_angle(move_ptr, 1 - 1);
	double q_4 = current_angle(move_ptr, 4 - 1);
	double q_5 = current_angle(move_ptr, 5 - 1);
	#ifdef DEBUG
		printf("Angles are:		%f %f %f %f %f\n", q_1, q_2, q_3, q_4, q_5);
	#endif // DEBUG

	/*	Transform angles into DH	*/
	double q1, q2, q3, q4, q5, q6;
	q1 = -pi - q_1;
	q2 = -(pi / 2.0) + (q_2 + pi);
	q3 = (pi / 2.0) + (q_3 + pi);
	q4 = q_4;
	q5 = -pi + q_5;
	q6 = 0;
	
	/*	Compute the position of the end-effector for these angles	*/
	double T[3];
	
	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;

	/*	Store the angles used in an array that can be passed around	*/
	angles[0] = (q_1);
	angles[1] = (q_2);
	angles[2] = (q_3);
	angles[3] = (q_4);
	angles[4] = (q_5);
	angles[5] = (0.0);

	/*	ensure the position is in m not mm for this next part	*/
	T[0] = (T[0] / 1000.0);
	T[1] = (T[1] / 1000.0);
	T[2] = (T[2] / 1000.0);
	#ifdef DEBUG
		printf("Calc position of Tip at:		%f %f %f\n", T[0], T[1], T[2]);
	#endif // DEBUG

	/*	store the calculated tip position in an array to be passed around	*/
	position[0] = (float)(T[0]);
	position[1] = (float)(T[1]);
	position[2] = (float)(T[2]);

	/*	Check if an error occurred in calculation, i.e. tip changed quadrants */
	if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
		#ifdef DEBUG
				printf("GGG");
		
		#endif //

		return 0;	// no error

	}
	else {
		#ifdef DEBUG
				printf("HHH");
		#endif // DEBUG
		return 2;	// error
	}

}


/*	Depreciated function for solving the joint angles of a 3R arm
*	@brief: Approximates the angles q1, q2, and q3 for the arm, to reach
*			the desired position.
*	@param: 3 floating point numbers specifying desired position,
*			and an array of angles to be updated
**/
void approximate_angles_kinematics(float x, float y, float z, double* angles) {

	/*	setup the variables for this method	*/
	double pi = 3.141594;
	double px = (double)((x)*1.0);
	double py = (double)((y)*1.0);
	double pz = (double)(z)*1.0;
	double d1 = 197.13;
	double d2 = 410.0;
	double d3 = 207.3;
	double e2 = 9.8;


	double j = pow(px, 2) + pow((pz - d1), 2) - (d2*d2) - (d3*d3);
	double k = 2.0 * d2 * d3;

	double q_3 = 1 * acos(j / k);

	double cq_2 = (px*(d2 + d3 * cos(q_3)) + d3 * sin(q_3)*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
	double sq_2 = (-px * d3*sin(q_3) + (d2 + d3 * cos(q_3))*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
	double q_2 = (pi / 2) - atan2(sq_2, cq_2);

	double q_1 = atan2(py, px);// -asin(e2 / (pow((pow(px, 2) + pow(py, 2)), .5)));

#ifdef DEBUG
	printf("this is q_1: %f ", q_1);
#endif
	angles[0] = pi - q_1;					//-1.0 * q_1;
	angles[1] = -(pi / 2.0) + (q_2 + pi);	//q_2;
	angles[2] = q_3 + (3.0 * pi / 2.0);

#ifdef DEBUG
	printf("after: %f\n", -pi - q_1);
#endif

}


/*	
*	@brief: approximates the position of the tip from the joint angles given
*
*	@param: array of 6 doubles containing the angles to move, and an array of doubles
*			to store the position output
*/
void get_position_from_angles(double* angles, double* positionVect) {

	double q1 = angles[0];
	double q2 = angles[1];
	double q3 = angles[2];
	double q4 = angles[3];
	double q5 = angles[4];
	double q6 = angles[5];

	double x_a = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	double y_a = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	double z_a = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;


	positionVect[0] = x_a;
	positionVect[1] = y_a;
	positionVect[2] = z_a;

}


/*
	@brief: Uses a control loop to move the angles of the Jaco arm to reach
			a desired position, specified by the current position of the end-
			effector + x,y,z function inputs. The mode specifies which gains to
			use, for moving forwards/backwards or up/down

	@param: struct containing scene information, struct containing Jaco arm information,
				a change in x,y,z coorinates of the end-effector
*/
void control_kinematics_v2_loop(info* info_ptr, move* move_ptr, float x, float y, float z) {

	double pi = 3.141592;

	/*	Get the joint4 position so q_2 and q_3 can be determined	*/
	float J4_desired[4];
	float J5_desired[4];
	J4_desired[0] = 0; J4_desired[1] = 0; J4_desired[2] = 0; J4_desired[3] = 0;
	J5_desired[0] = 0; J5_desired[1] = 0; J5_desired[2] = 0; J5_desired[3] = 0;
	int joint4Handle = info_ptr->jacoArmJointHandles[4 - 1]; 
	int joint5Handle = info_ptr->jacoArmJointHandles[5 - 1];
	get_world_position_vrep(info_ptr, J4_desired, joint4Handle);
	get_world_position_vrep(info_ptr, J5_desired, joint5Handle);

	J4_desired[0] -= info_ptr->armPosition[0];
	J4_desired[1] -= info_ptr->armPosition[1];
	J5_desired[0] -= info_ptr->armPosition[0];
	J5_desired[1] -= info_ptr->armPosition[1];

	/*	Get the current tip position and update S_desired	*/
	float S_desired[4];

	S_desired[0] = 0; S_desired[1] = 0; S_desired[2] = 0; S_desired[3] = 0;
	int tipHandle = info_ptr->targetHandle - 1;
	get_world_position_vrep(info_ptr, S_desired, tipHandle);

	printf("\nActual Tip Position:	%f %f %f\n", S_desired[0] - info_ptr->armPosition[0], S_desired[1] - info_ptr->armPosition[1], S_desired[2]);

	/*	Calculate the desired end-effector position	*/
	S_desired[0] = (x - info_ptr->armPosition[0] + S_desired[0]) * 1000.0;
	S_desired[1] = (y - info_ptr->armPosition[1] + S_desired[1]) * 1000.0;
	S_desired[2] = (S_desired[2] + z) * 1000.0;

	printf("\nDesired Tip Position:	%f %f %f\n", S_desired[0], S_desired[1], S_desired[2]);


	/*	Start of control sequence	*/

	double desHypot = pow(S_desired[0] * S_desired[0] + S_desired[1] * S_desired[1], 0.5);

	double S_D1 = (double)(S_desired[1]);
	double S_D0 = (double)(S_desired[0]);
	double S_D2 = (double)(S_desired[2]);


	double desiredAngXY = atan2(S_D1, S_D0);
	double desiredAngZ = atan2(S_D2, desHypot);

	double actHypot = 0;
	double actAngXY = desiredAngXY;
	double actAngZ = desiredAngZ;

	double accumError[6] = { 0, 0, 0, 0, 0, 0 };

	double error[6] = { 0, 0, 0, 0, 0, 0 };

	double angles[6] = { 0, 0, 0, 0, 0, 0 };

	double actualPos[3] = { 0, 0, 0 };

	double angles1[6] = { 0, 0, 0, 0, 0, 0 };

	/*	For a local solution, an approximation of angles needs to be given.
		Considering the change in end-effector is small, the best approximation
		is the current approximation. Otherwise the angles could be calculated 
		using:
		approximate_angles_kinematics(S_desired[0], S_desired[1], S_desired[2], angles);
	*/
	
	angles[0] = -pi - current_angle(move_ptr, 0);
	angles[1] = -(pi / 2.0) + (current_angle(move_ptr, 1) + pi);
	angles[2] = (pi / 2.0) + (current_angle(move_ptr, 2) + pi);
	angles[3] = current_angle(move_ptr, 3);
	angles[4] = -pi + current_angle(move_ptr, 4);
	angles[5] = pi;

	#ifdef DEBUG
		printf("proposed angles are:	%f %f %f\n", angles[0], angles[1], angles[2]);

		printf("desired angles:		XY: %f	Z: %f\n", desiredAngXY, desiredAngZ);

	#endif
	/*	Setup file for writing control loop data	*/
	/*/////////////////// 
	
	FILE* object_fp = fopen("pid_data_extreme.txt", "w+");
	if (object_fp == NULL) {
		fprintf(stdout, "Failed to generate file\n");
		exit(1);
	}


	char line1[256];
	sprintf(line1, "errorXY, errorZ, posX, posY, posZ, q_1, q_2, q_3, q_4, q_5, q_6\n");
	fputs(line1, object_fp);
	*/

	clock_t start = clock();

	/*	begin control sequence for 400 iterations	*/
	for (int loop = 0; loop < 150; loop++) {

		/*	Calculate error in X and Y, and XY and Z planes	*/
		double errorAngXY = desiredAngXY - actAngXY;
		double errorAngZ = desiredAngZ - actAngZ;
		double q_1, q_2, q_3;
		
		/*	Calculate the angles to input	*/
		/*	joint 1	*/
		error[0] = errorAngXY * 0.03;
		accumError[0] = (accumError[0] + errorAngXY) * 0.99;

		q_1 = fmod(error[0] + accumError[0], 2.0 * pi) + angles[0];

		/*	joint 2	*/
		error[1] = 0.55 * errorAngZ;
		accumError[1] = (accumError[1] + errorAngZ) * 0.51;

		q_2 = error[1] + accumError[1] + angles[1];

		/*	joint 3	*/
		error[2] = 1.10 * errorAngZ;
		accumError[2] = (accumError[2] + errorAngZ) * 1.01;

		q_3 = error[2] + accumError[2] + angles[2];

		/*	joint 4	*/
		error[3] = (0.05 * errorAngXY + 0.05 * errorAngZ) * 0.1;
		accumError[3] = (accumError[3] + 0.05 * errorAngXY + 0.05 * errorAngZ) * 0.02;

		double q_4 = error[3] + accumError[3] + angles[3];

		/*	joint 5	*/
		error[4] = (0.05 * errorAngXY + 0.05 * errorAngZ) * 0.15;
		accumError[4] = (accumError[4] + 0.05 * errorAngXY + 0.05 * errorAngZ) * 0.150;
		double q_5 = error[4] + accumError[4] + angles[4];


		/*	Calculate new position from angles	*/
		angles1[0] = q_1;
		angles1[1] = q_2;
		angles1[2] = q_3;
		angles1[3] = q_4;
		angles1[4] = q_5;
		angles1[5] = move_ptr->currAng[5];
		
		get_position_from_angles(angles1, actualPos);

		/*	Calculate the new plane positions	*/
		actHypot = pow(actualPos[0] * actualPos[0] + actualPos[1] * actualPos[1], 0.5);

		actAngXY = atan2(actualPos[1], actualPos[0]);
		actAngZ = atan2(actualPos[2], actHypot);

		#ifdef DEBUG
			if (!(loop % 40)) {
				printf("%f %f	%f %f %f\n", errorAngXY, errorAngZ, actualPos[0], actualPos[1], actualPos[2]);
				printf("%f  %f  %f  %f  %f\n", q_1, q_2, q_3, q_4, q_5);
				printf("Act:	%f %f	Q_3 Error	%f	Accum	%f\n", actAngXY, actAngZ, error[2], accumError[2]);
			}
			/*	Write the loop data/information to a file	*/
			/*
			char line2[256];
			sprintf(line2, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", errorAngXY, errorAngZ, actualPos[0], actualPos[1], actualPos[2], q_1, q_2, q_3, q_4, q_5, q_6);
			fputs(line2, object_fp);
			*/

			//if (loop == 0) {
			//	printf("#%f %f		%f %f %f\n", errorAngXY, errorAngZ, actualPos[0], actualPos[1], actualPos[2]);
			//}
		#endif

	}
	
	// close the file after writing
	/*
	fflush(object_fp);
	fclose(object_fp);
	*/

	clock_t end = clock();
	
	for (int ang = 0; ang < 6; ang++) {
		if (angles1[ang] > 2 * 3.141592) {
			angles1[ang] = fmod(angles1[ang], 2 * 3.141592);
		}
		else if (angles1[ang] < (-2 * 3.141592)) {
			angles1[ang] = fmod(angles1[ang], -2 * 3.141592);
		}
	}

	#ifdef DEBUG
		printf("\nChange Ang: %f %f %f %f %f %f\n", angles[0] - angles1[0], angles[1] - angles1[1], angles[2] - angles1[2], angles[3] - angles1[3], angles[4] - angles1[4], angles[5] - angles1[5]);
	#endif
	
	/*	Got angles, move the arm	*/
	pause_communication_vrep(info_ptr, 1);
	set_joint_angle_vrep(info_ptr, move_ptr, 1, -angles1[0] - pi);					// -2 * pi);	//(3.141592 / 2.0) - 3.141592);
	set_joint_angle_vrep(info_ptr, move_ptr, 2, angles1[1] + pi/2.0 - pi);			// - pi/2.0); // +pi / 2.0 - pi);				//angles1[1] + pi - pi);//(pi / 2) - pi);
	set_joint_angle_vrep(info_ptr, move_ptr, 3, angles1[2] -3.0*pi / 2.0);			// -pi - pi / 2.0);//(3.0 * pi / 2.0));
	set_joint_angle_vrep(info_ptr, move_ptr, 4, angles1[3]);						//angles1[3]);
	set_joint_angle_vrep(info_ptr, move_ptr, 5, angles1[4] +pi);
	pause_communication_vrep(info_ptr, 0);

}


/*
*	@brief calculates the desired tip position using pid control on each joint
*	@param the information and movement structures with a change in coordinates to move
*	@ret none
*/
void control_kinematics_v2(info* info_ptr, move* move_ptr, float x, float y, float z) {

	// This loop can be run numerious times to increase moving distance
	control_kinematics_v2_loop(info_ptr, move_ptr, x, y, z);
}


/*	
*	@brief calculates the desired tip position and localises the joint4 position
*	to acheive this when the mode is 1, when the mode is 2, a control loop
*	implementation is used, and when the mode is set to 3, the jacobian
*	for the current joint angles is calculated
*	@param the information and movement structures with a change in coordinates to move
*	@ret none
*/
void control_kinematics(info* info_ptr, move* move_ptr, float x, float y, float z) {

	/*	Should be able to use any mode, default is this function	*/
	if (Mode2) {
		control_kinematics_v2(info_ptr, move_ptr, x, y, z);
		return;
	}	// IK_MODE2
	if (Mode3) {
		control_kinematics_v3(info_ptr, move_ptr, x, y, z);
		return;
	}	// IK_MODE3


	clock_t start = clock();

	/*	Get the joint4 position so q_2 and q_3 can be determined	*/
	float J4_desired[4];
	float J5_desired[4];
	J4_desired[0] = 0; J4_desired[1] = 0; J4_desired[2] = 0; J4_desired[3] = 0;
	J5_desired[0] = 0; J5_desired[1] = 0; J5_desired[2] = 0; J5_desired[3] = 0;
	int joint4Handle = info_ptr->jacoArmJointHandles[4 - 1];//27;
	int joint5Handle = info_ptr->jacoArmJointHandles[5 - 1];
	get_world_position_vrep(info_ptr, J4_desired, joint4Handle);
	get_world_position_vrep(info_ptr, J5_desired, joint5Handle);

	J4_desired[0] -= info_ptr->armPosition[0];
	J4_desired[1] -= info_ptr->armPosition[1];
	J5_desired[0] -= info_ptr->armPosition[0];
	J5_desired[1] -= info_ptr->armPosition[1];


	/*	Get the current tip position and update S_desired	*/
	float S_desired[4];

	S_desired[0] = 0; S_desired[1] = 0; S_desired[2] = 0; S_desired[3] = 0;
	int tipHandle = info_ptr->targetHandle -1;
	get_world_position_vrep(info_ptr, S_desired, tipHandle);

	printf("\nActual Tip Position:	%f %f %f\n", S_desired[0] - info_ptr->armPosition[0] , S_desired[1] - info_ptr->armPosition[1], S_desired[2]);

	S_desired[0] = x - info_ptr->armPosition[0] + S_desired[0];
	S_desired[1] = y - info_ptr->armPosition[1] + S_desired[1];
	S_desired[2] += z;

	J4_desired[0] = x + (J4_desired[0] + J5_desired[0]) / 2.0;
	J4_desired[1] = y + (J4_desired[1] + J5_desired[1]) / 2.0;
	J4_desired[2] = z + (J4_desired[2] + J5_desired[2]) / 2.0;
	
	printf("\nDesired Tip Position:	%f %f %f\n", S_desired[0], S_desired[1], S_desired[2]);
	printf("\nJoint 4 Position:	%f %f %f\n", J4_desired[0], J4_desired[1], J4_desired[2]);
	
	/*	Initialise the control variables	*/
	float S_error[4] = { 0, 0, 0, 0 };
	float S_accumError[4] = { 0, 0, 0, 0 };
	double initialAngles[6] = { current_angle(move_ptr, 0), current_angle(move_ptr, 1), current_angle(move_ptr, 2), current_angle(move_ptr, 3), 0.0, 0.0 };
	double angles[6] = { 0, 0, 0, 0, 0, 0 };
	int loop = 1;

	#ifdef DEBUG
		printf("calculating");
	#endif // DEBUG

	/*	open a file for writing loop data	*/
	/*
	FILE* object_fp = fopen("pi_data.txt", "w+");
	if (object_fp == NULL) {
		fprintf(stdout, "Failed to generate file\n");
		exit(1);
	}


	char line1[256];
	sprintf(line1, "errorX errorY errorZ accumErrorX accumErrorY accumErrorZ PosX PosY PosZ desX desY desZ\n");
	fputs(line1, object_fp);
	*/

	/*	loop through, correcting for error in end-effector position	*/
	for(int run = 0; run < 20; run++) {

		/*	Update the desired Joint4 position	*/
		float position[4] = { J4_desired[0] + S_error[0] + S_accumError[0], J4_desired[1] + S_error[1] + S_accumError[1], J4_desired[2] + S_error[2] + S_accumError[2], 0.0 };

		/*	update position with the calculated tip position 	*/
		int errorCheck = inverse_kinematics(move_ptr, info_ptr, position, angles);

		if (errorCheck == 1) {
			printf("IK ERROR %d\n", errorCheck);
			break;
		}

		else if (errorCheck == 2) {
			if ((S_desired[0] / position[0] < 0) || (S_desired[1] / position[1] < 0)) {
				// coordinate frame changed... leave this loop and use the least valid angles
				angles[0] = initialAngles[0];
				angles[1] = initialAngles[1];
				angles[2] = initialAngles[2];
				angles[3] = initialAngles[3];
				angles[4] = initialAngles[4];
				angles[5] = initialAngles[5];
				break;
			}
		}
		else {
			initialAngles[0] = angles[0];
			initialAngles[1] = angles[1];
			initialAngles[2] = angles[2];
			initialAngles[3] = angles[3];
			initialAngles[4] = angles[4];
			initialAngles[5] = angles[5];
		}

		/*	check that the position is within 2cm of the desired tip position	*/
		if ((fabs(S_desired[0] - position[0]) < 0.006) && (fabs(S_desired[1] - position[1]) < 0.006) 
				&& (fabs(S_desired[2] - position[2]) < 0.006)) {
			loop = 0;

			/*	write data from this loop to the file	*/
			/*
			char line3[256];
			sprintf(line3, "%f %f %f %f %f %f %f %f %f %f %f %f\n", S_error[0], S_error[1], S_error[2], S_accumError[0], S_accumError[1], S_accumError[2], position[0], position[1], position[2], S_desired[0], S_desired[1], S_desired[2]);
			fputs(line3, object_fp);
			*/
			// exit the loop as the desired accuracy has been achieved
			break;
		}

		/*	Add some proportionality to the error to ensure limited overshoot
		*	between calculations
		*/
		S_accumError[0] += 0.2*(S_desired[0] - position[0]);
		S_accumError[1] += 0.2*(S_desired[1] - position[1]);
		S_error[0] = 0.08*(S_desired[0] - position[0]);
		S_error[1] = 0.08*(S_desired[1] - position[1]);

		if (((current_angle(move_ptr, 1) < 0.01) || (current_angle(move_ptr, 1) > 6.24)) &&
			((current_angle(move_ptr, 2) < 0.01) || (current_angle(move_ptr, 2) > 6.24))) {
			S_desired[2] = position[2];
			// arm is fully extended
			// there's no hope of reaching a higher arm position from joint4 pos
		}
		else {
			S_accumError[2] += 0.1*(S_desired[2] - position[2]);
			S_error[1] = 0.08*(S_desired[1] - position[1]);
		}
		#ifdef DEBUG
			printf("S_error:	%f %f %f\n", S_error[0], S_error[1], S_error[2]);
		#endif // DEBUG

		/*	write data from this loop to the file	*/
		/*
		char line2[256];
		sprintf(line2, "%f %f %f %f %f %f %f %f %f %f %f %f\n", S_error[0], S_error[1], S_error[2], S_accumError[0], S_accumError[1], S_accumError[2], position[0], position[1], position[2], S_desired[0], S_desired[1], S_desired[2]);
		fputs(line2, object_fp);
		*/


	}

	/*	close the file that contains data	*/
	/*
	fflush(object_fp);
	fclose(object_fp);
	*/

	clock_t end = clock();

	printf("Calculated INV K in %ld ms\n", end - start);

	/*	Got angles, move the arm	*/
	pause_communication_vrep(info_ptr, 1);
	set_joint_angle_vrep(info_ptr, move_ptr, 2, angles[1]);
	set_joint_angle_vrep(info_ptr, move_ptr, 3, angles[2]);
	pause_communication_vrep(info_ptr, 0);
}



/* Updates current position vector, representing the calculated
*	coordinates of the arm, using modified DH parameters
*  and forward kinematics */
void fk_mod(move* move_ptr) {
	
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



/*
*	@brief calculates the desired tip position and localises the joint4 position
*	to acheive this using the Jacobian matrix at the current angle position, and 
*	calculating the change in angles required for a small change in position
*	@param the information and movement structures with a change in coordinates to move
*	@ret none
*/
void control_kinematics_v3(info* info_ptr, move* move_ptr, float x, float y, float z) {

	/*	Need to make x,y,z * 1000	*/

	/*	angles from the Jaco arm in V-REP, offset for transformation matrix	*/
	double pi = 3.141592;
	double q1 = -pi - move_ptr->currAng[0];
	double q2 = (-pi/2.0) + pi + move_ptr->currAng[1];
	double q3 = (pi/2.0) + pi + move_ptr->currAng[2];
	double q4 = move_ptr->currAng[3];
	double q5 = -pi + move_ptr->currAng[4];
	double q6 = pi;

	printf("Q: %f %f %f %f %f %f\n", q1, q2, q3, q4, q5, q6);

	/*	Compute JJ^T given the set of angles - the output of this is correct	*/
	double J_JT[5][5];
	J_JT[1 - 1][1 - 1] = pow(sin(q1)*-9.8 - cos(q1)*cos(q2)*4.1E2 + cos(q4)*sin(q1)*1.619070323027551E2 - sin(q1)*sin(q4)*sin(q5)*1.756140646055102E2 + cos(q2 - q3)*cos(q1)*sin(q4)*1.619070323027551E2 + cos(q1)*cos(q2)*sin(q3)*3.435587236306403E2 - cos(q1)*cos(q3)*sin(q2)*3.435587236306403E2 + cos(q4)*cos(q5)*sin(q1)*8.780703230275508E1 - cos(q1)*cos(q2)*cos(q5)*sin(q3)*1.520862412102134E2 + cos(q1)*cos(q3)*cos(q5)*sin(q2)*1.520862412102134E2 + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*1.756140646055102E2 + cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4)*8.780703230275508E1 + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*1.756140646055102E2 + cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*8.780703230275508E1, 2.0) + pow(cos(q1)*9.8 - cos(q1)*cos(q4)*1.619070323027551E2 - cos(q2)*sin(q1)*4.1E2 + cos(q2 - q3)*sin(q1)*sin(q4)*1.619070323027551E2 - cos(q1)*cos(q4)*cos(q5)*8.780703230275508E1 + cos(q2)*sin(q1)*sin(q3)*3.435587236306403E2 - cos(q3)*sin(q1)*sin(q2)*3.435587236306403E2 + cos(q1)*sin(q4)*sin(q5)*1.756140646055102E2 - cos(q2)*cos(q5)*sin(q1)*sin(q3)*1.520862412102134E2 + cos(q3)*cos(q5)*sin(q1)*sin(q2)*1.520862412102134E2 + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*1.756140646055102E2 + cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*8.780703230275508E1 + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*1.756140646055102E2 + cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*8.780703230275508E1, 2.0);

	J_JT[1 - 1][2 - 1] = sin(q2)*-4.018E3 + cos(q2)*cos(q3)*3.366875491580275E3 + cos(q4)*sin(q2)*6.638188324412958E4 + sin(q2)*sin(q3)*3.366875491580275E3 - sin(q2)*sin(q4)*sin(q5)*7.200176648825917E4 - cos(q2)*cos(q3)*cos(q4)*5.562457336475938E4 - cos(q2)*cos(q3)*cos(q5)*1.490445163860092E3 + cos(q4)*cos(q5)*sin(q2)*3.600088324412958E4 - cos(q2)*sin(q3)*sin(q4)*1.586688916567E3 + cos(q3)*sin(q2)*sin(q4)*1.586688916567E3 - cos(q4)*sin(q2)*sin(q3)*5.562457336475938E4 - cos(q2)*sin(q3)*sin(q5)*2.843315203090245E4 + cos(q3)*sin(q2)*sin(q5)*2.843315203090245E4 - cos(q5)*sin(q2)*sin(q3)*1.490445163860092E3 + cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*1.335424149474981E4 + cos(q2)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*5.68663040618049E4 - cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*5.68663040618049E4 + cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.335424149474981E4 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*5.543039975302317E3 - cos(q2)*cos(q4)*sin(q3)*sin(q4)*4.626412578182928E3 + cos(q3)*cos(q4)*sin(q2)*sin(q4)*4.626412578182928E3 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*6.033374388745788E4 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.721017833134E3 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*8.605089165669998E2 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.721017833134E3 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*8.605089165669998E2 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*5.543039975302317E3 - cos(q2)*cos(q5)*sin(q3)*sin(q5)*1.542014984363415E4 + cos(q3)*cos(q5)*sin(q2)*sin(q5)*1.542014984363415E4 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q4)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*3.855037460908537E4 - cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*3.855037460908537E4 + cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*3.08402996872683E4 - cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*3.08402996872683E4 + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q4)*2.843315203090245E4 - cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q4)*2.843315203090245E4 - cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.670848298949963E4 - cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.670848298949963E4;

	J_JT[1 - 1][3 - 1] = cos(q2)*cos(q3)*-3.366875491580275E3 - sin(q2)*sin(q3)*3.366875491580275E3 + cos(q2)*cos(q3)*cos(q4)*5.562457336475938E4 + cos(q2)*cos(q3)*cos(q5)*1.490445163860092E3 + cos(q2)*sin(q3)*sin(q4)*1.586688916567E3 - cos(q3)*sin(q2)*sin(q4)*1.586688916567E3 + cos(q4)*sin(q2)*sin(q3)*5.562457336475938E4 + cos(q2)*sin(q3)*sin(q5)*2.843315203090245E4 - cos(q3)*sin(q2)*sin(q5)*2.843315203090245E4 + cos(q5)*sin(q2)*sin(q3)*1.490445163860092E3 - cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*1.335424149474981E4 - cos(q2)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*5.68663040618049E4 + cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*5.68663040618049E4 - cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.335424149474981E4 + cos(q2)*cos(q3)*cos(q4)*cos(q5)*5.543039975302317E3 + cos(q2)*cos(q4)*sin(q3)*sin(q4)*4.626412578182928E3 - cos(q3)*cos(q4)*sin(q2)*sin(q4)*4.626412578182928E3 - cos(q2)*cos(q3)*sin(q4)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.721017833134E3 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*8.605089165669998E2 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.721017833134E3 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*8.605089165669998E2 + cos(q4)*cos(q5)*sin(q2)*sin(q3)*5.543039975302317E3 + cos(q2)*cos(q5)*sin(q3)*sin(q5)*1.542014984363415E4 - cos(q3)*cos(q5)*sin(q2)*sin(q5)*1.542014984363415E4 - sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.033374388745788E4 - cos(q2)*cos(q4)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*3.855037460908537E4 + cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*3.855037460908537E4 - cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*3.08402996872683E4 + cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*3.08402996872683E4 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q4)*2.843315203090245E4 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q4)*2.843315203090245E4 + cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.670848298949963E4 + cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.670848298949963E4;

	J_JT[1 - 1][4 - 1] = cos(q2)*cos(q3)*5.705418679635367E4 - cos(q2)*sin(q4)*6.638188324412958E4 + sin(q2)*sin(q3)*5.705418679635367E4 - cos(q2)*cos(q3)*pow(cos(q5), 2.0)*2.313022476545122E4 - pow(cos(q5), 2.0)*sin(q2)*sin(q3)*2.313022476545122E4 - cos(q2)*cos(q3)*cos(q4)*1.586688916567E3 + cos(q2)*cos(q3)*cos(q5)*2.843315203090245E4 - cos(q2)*cos(q4)*sin(q5)*7.200176648825917E4 - cos(q2)*cos(q5)*sin(q4)*3.600088324412958E4 + cos(q2)*sin(q3)*sin(q4)*5.562457336475938E4 - cos(q3)*sin(q2)*sin(q4)*5.562457336475938E4 - cos(q4)*sin(q2)*sin(q3)*1.586688916567E3 + cos(q5)*sin(q2)*sin(q3)*2.843315203090245E4 - cos(q2)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*1.335424149474981E4 + cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*1.335424149474981E4 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*8.605089165669998E2 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.721017833134E3 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*5.543039975302317E3 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*6.033374388745788E4 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*5.543039975302317E3 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*8.605089165669998E2 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.721017833134E3 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.670848298949963E4 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.670848298949963E4;

	J_JT[1 - 1][5 - 1] = cos(q2)*cos(q3)*1.542014984363415E4 + sin(q2)*sin(q3)*1.542014984363415E4 + cos(q2)*cos(q3)*cos(q5)*2.843315203090245E4 - cos(q2)*cos(q4)*sin(q5)*3.600088324412958E4 - cos(q2)*cos(q5)*sin(q4)*7.200176648825917E4 - cos(q2)*sin(q3)*sin(q4)*2.670848298949963E4 + cos(q3)*sin(q2)*sin(q4)*2.670848298949963E4 - cos(q2)*sin(q3)*sin(q5)*1.490445163860092E3 + cos(q3)*sin(q2)*sin(q5)*1.490445163860092E3 + cos(q5)*sin(q2)*sin(q3)*2.843315203090245E4 - cos(q2)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*1.0E-28 + cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*1.0E-28 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*1.721017833134E3 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*8.605089165669998E2 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*5.479070391215557E4 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*6.033374388745788E4 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*5.479070391215557E4 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*6.033374388745788E4 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*1.721017833134E3 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*8.605089165669998E2 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.659964638814754E-29 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.659964638814754E-29;
	
	J_JT[2 - 1][1 - 1] = sin(q2)*-4.018E3 + cos(q2)*cos(q3)*3.366875491580275E3 + cos(q4)*sin(q2)*6.638188324412958E4 + sin(q2)*sin(q3)*3.366875491580275E3 - sin(q2)*sin(q4)*sin(q5)*7.200176648825917E4 - cos(q2)*cos(q3)*cos(q4)*5.562457336475938E4 - cos(q2)*cos(q3)*cos(q5)*1.490445163860092E3 + cos(q4)*cos(q5)*sin(q2)*3.600088324412958E4 - cos(q2)*sin(q3)*sin(q4)*1.586688916567E3 + cos(q3)*sin(q2)*sin(q4)*1.586688916567E3 - cos(q4)*sin(q2)*sin(q3)*5.562457336475938E4 - cos(q2)*sin(q3)*sin(q5)*2.843315203090245E4 + cos(q3)*sin(q2)*sin(q5)*2.843315203090245E4 - cos(q5)*sin(q2)*sin(q3)*1.490445163860092E3 + cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*1.335424149474981E4 + cos(q2)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*5.68663040618049E4 - cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*5.68663040618049E4 + cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.335424149474981E4 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*5.543039975302317E3 - cos(q2)*cos(q4)*sin(q3)*sin(q4)*4.626412578182928E3 + cos(q3)*cos(q4)*sin(q2)*sin(q4)*4.626412578182928E3 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*6.033374388745788E4 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.721017833134E3 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*8.605089165669998E2 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.721017833134E3 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*8.605089165669998E2 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*5.543039975302317E3 - cos(q2)*cos(q5)*sin(q3)*sin(q5)*1.542014984363415E4 + cos(q3)*cos(q5)*sin(q2)*sin(q5)*1.542014984363415E4 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q4)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*3.855037460908537E4 - cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*3.855037460908537E4 + cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*3.08402996872683E4 - cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*3.08402996872683E4 + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q4)*2.843315203090245E4 - cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q4)*2.843315203090245E4 - cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.670848298949963E4 - cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.670848298949963E4;

	J_JT[2 - 1][2 - 1] = pow(cos(q1), 2.0)*pow(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32, 2.0)*4.0E-62 + pow(sin(q1), 2.0)*pow(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32, 2.0)*4.0E-62 + pow(sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1), 2.0);

	J_JT[2 - 1][3 - 1] = cos(q5)*7.606795779302279E4 + sin(q3)*1.408590766885625E5 + cos(q3)*sin(q4)*6.638188324412958E4 - cos(q5)*sin(q3)*6.235535889618751E4 - pow(cos(q4), 2.0)*4.626412578182928E3 - pow(cos(q5), 2.0)*3.08402996872683E4 + pow(cos(q4), 2.0)*cos(q5)*2.843315203090245E4 - pow(cos(q2), 2.0)*pow(cos(q5), 2.0)*8.0E-29 - pow(cos(q3), 2.0)*pow(cos(q5), 2.0)*8.0E-29 + pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*3.855037460908537E4 - pow(cos(q2), 2.0)*cos(q5)*sin(q3)*2.0E-28 + cos(q3)*cos(q4)*sin(q5)*7.200176648825917E4 + cos(q3)*cos(q5)*sin(q4)*3.600088324412958E4 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*pow(cos(q5), 2.0)*1.6E-28 - pow(cos(q2), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*2.0E-29 - pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*2.0E-29 - cos(q4)*sin(q4)*sin(q5)*5.68663040618049E4 + pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q4)*1.0E-28 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*4.0E-29 + cos(q2)*cos(q3)*cos(q5)*sin(q2)*2.0E-28 + cos(q2)*cos(q3)*sin(q2)*sin(q3)*1.0E-27 - cos(q2)*cos(q5)*sin(q2)*sin(q4)*1.0E-28 + cos(q3)*cos(q5)*sin(q3)*sin(q4)*1.0E-28 - cos(q4)*cos(q5)*sin(q4)*sin(q5)*3.08402996872683E4 - cos(q2)*cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*pow(cos(q3), 2.0)*cos(q5)*sin(q2)*sin(q4)*2.0E-28 - pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*2.0E-28 - cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*1.0E-28 - cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*1.0E-28 + cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q3)*1.0E-28 - cos(q2)*pow(cos(q3), 2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.0E-28 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0E-28 - 1.442464836918001E5;

	J_JT[2 - 1][4 - 1] = sin(q1)*(cos(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - cos(q1)*(sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1))*1.0E-31;

	J_JT[2 - 1][5 - 1] = -(cos(q2 - q3)*sin(q5)*1.520862412102134E2 + sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 - sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1)*(sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1)) + sin(q1)*(sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2 + cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - cos(q1)*(cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31;
	
	J_JT[3 - 1][1 - 1] = cos(q2)*cos(q3)*-3.366875491580275E3 - sin(q2)*sin(q3)*3.366875491580275E3 + cos(q2)*cos(q3)*cos(q4)*5.562457336475938E4 + cos(q2)*cos(q3)*cos(q5)*1.490445163860092E3 + cos(q2)*sin(q3)*sin(q4)*1.586688916567E3 - cos(q3)*sin(q2)*sin(q4)*1.586688916567E3 + cos(q4)*sin(q2)*sin(q3)*5.562457336475938E4 + cos(q2)*sin(q3)*sin(q5)*2.843315203090245E4 - cos(q3)*sin(q2)*sin(q5)*2.843315203090245E4 + cos(q5)*sin(q2)*sin(q3)*1.490445163860092E3 - cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*1.335424149474981E4 - cos(q2)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*5.68663040618049E4 + cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*5.68663040618049E4 - cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.335424149474981E4 + cos(q2)*cos(q3)*cos(q4)*cos(q5)*5.543039975302317E3 + cos(q2)*cos(q4)*sin(q3)*sin(q4)*4.626412578182928E3 - cos(q3)*cos(q4)*sin(q2)*sin(q4)*4.626412578182928E3 - cos(q2)*cos(q3)*sin(q4)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.721017833134E3 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*8.605089165669998E2 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.721017833134E3 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*8.605089165669998E2 + cos(q4)*cos(q5)*sin(q2)*sin(q3)*5.543039975302317E3 + cos(q2)*cos(q5)*sin(q3)*sin(q5)*1.542014984363415E4 - cos(q3)*cos(q5)*sin(q2)*sin(q5)*1.542014984363415E4 - sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.033374388745788E4 - cos(q2)*cos(q4)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*3.855037460908537E4 + cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*3.855037460908537E4 - cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*3.08402996872683E4 + cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*3.08402996872683E4 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q4)*2.843315203090245E4 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q4)*2.843315203090245E4 + cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.670848298949963E4 + cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.670848298949963E4;

	J_JT[3 - 1][2 - 1] = cos(q5)*7.606795779302279E4 + sin(q3)*1.408590766885625E5 + cos(q3)*sin(q4)*6.638188324412958E4 - cos(q5)*sin(q3)*6.235535889618751E4 - pow(cos(q4), 2.0)*4.626412578182928E3 - pow(cos(q5), 2.0)*3.08402996872683E4 + pow(cos(q4), 2.0)*cos(q5)*2.843315203090245E4 - pow(cos(q2), 2.0)*pow(cos(q5), 2.0)*8.0E-29 - pow(cos(q3), 2.0)*pow(cos(q5), 2.0)*8.0E-29 + pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*3.855037460908537E4 - pow(cos(q2), 2.0)*cos(q5)*sin(q3)*2.0E-28 + cos(q3)*cos(q4)*sin(q5)*7.200176648825917E4 + cos(q3)*cos(q5)*sin(q4)*3.600088324412958E4 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*pow(cos(q5), 2.0)*1.6E-28 - pow(cos(q2), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*2.0E-29 - pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*2.0E-29 - cos(q4)*sin(q4)*sin(q5)*5.68663040618049E4 + pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q4)*1.0E-28 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*4.0E-29 + cos(q2)*cos(q3)*cos(q5)*sin(q2)*2.0E-28 + cos(q2)*cos(q3)*sin(q2)*sin(q3)*1.0E-27 - cos(q2)*cos(q5)*sin(q2)*sin(q4)*1.0E-28 + cos(q3)*cos(q5)*sin(q3)*sin(q4)*1.0E-28 - cos(q4)*cos(q5)*sin(q4)*sin(q5)*3.08402996872683E4 - cos(q2)*cos(q3)*pow(cos(q4), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*pow(cos(q3), 2.0)*cos(q5)*sin(q2)*sin(q4)*2.0E-28 - pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*2.0E-28 - cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*1.0E-28 - cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*1.0E-28 + cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q4), 2.0)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*1.0E-28 + cos(q2)*cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q3)*1.0E-28 - cos(q2)*pow(cos(q3), 2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.0E-28 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0E-28 - 1.442464836918001E5;

	J_JT[3 - 1][3 - 1] = pow(cos(q5)*(sin(q4)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*5.0E-1 + sin(q1)*sin(q2)*sin(q3)*8.660254037844386E-1 + cos(q2)*cos(q3)*sin(q1)*8.660254037844386E-1)*1.756140646055102E2 + sin(q4)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*1.619070323027551E2 - sin(q1)*sin(q2)*sin(q3)*3.435587236306403E2 + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*1.756140646055102E2 - cos(q2)*cos(q3)*sin(q1)*3.435587236306403E2, 2.0) + pow(cos(q5)*(sin(q4)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*5.0E-1 + cos(q1)*cos(q2)*cos(q3)*8.660254037844386E-1 + cos(q1)*sin(q2)*sin(q3)*8.660254037844386E-1)*1.756140646055102E2 + sin(q4)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*1.619070323027551E2 + cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*1.756140646055102E2 - cos(q1)*cos(q2)*cos(q3)*3.435587236306403E2 - cos(q1)*sin(q2)*sin(q3)*3.435587236306403E2, 2.0) + pow(sin(q2 - q3)*-3.435587236306403E2 + cos(q2 - q3)*sin(q4)*1.619070323027551E2 + sin(q2 - q3)*cos(q5)*1.520862412102134E2 + cos(q2 - q3)*cos(q4)*sin(q5)*1.756140646055102E2 + cos(q2 - q3)*cos(q5)*sin(q4)*8.780703230275508E1, 2.0);

	J_JT[3 - 1][4 - 1] = sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(sin(q2 - q3)*-3.435587236306403E2 + cos(q2 - q3)*sin(q4)*1.619070323027551E2 + sin(q2 - q3)*cos(q5)*1.520862412102134E2 + cos(q2 - q3)*cos(q4)*sin(q5)*1.756140646055102E2 + cos(q2 - q3)*cos(q5)*sin(q4)*8.780703230275508E1)*-1.0E-31 + sin(q1)*(cos(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) + cos(q4)*sin(q1)*sin(q2)*sin(q3))*8.780703230275508E1 - sin(q5)*(-cos(q1)*cos(q4) + cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4))*1.756140646055102E2 + cos(q2 - q3)*cos(q4)*sin(q1)*1.619070323027551E2)*(cos(q2)*cos(q3)*8.588968090766008E63 + sin(q2)*sin(q3)*8.588968090766008E63 - cos(q2)*cos(q3)*cos(q5)*3.802156030255336E63 - cos(q2)*sin(q3)*sin(q4)*4.047675807568877E63 + cos(q3)*sin(q2)*sin(q4)*4.047675807568877E63 - cos(q5)*sin(q2)*sin(q3)*3.802156030255336E63 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.390351615137754E63 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*2.195175807568877E63 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.390351615137754E63 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*2.195175807568877E63)*4.0E-62 - cos(q1)*(sin(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4))*1.756140646055102E2 - cos(q5)*(sin(q1)*sin(q4)*-1.0 + cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3))*8.780703230275508E1 - cos(q2 - q3)*cos(q1)*cos(q4)*1.619070323027551E2)*(cos(q2)*cos(q3)*8.588968090766008E63 + sin(q2)*sin(q3)*8.588968090766008E63 - cos(q2)*cos(q3)*cos(q5)*3.802156030255336E63 - cos(q2)*sin(q3)*sin(q4)*4.047675807568877E63 + cos(q3)*sin(q2)*sin(q4)*4.047675807568877E63 - cos(q5)*sin(q2)*sin(q3)*3.802156030255336E63 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.390351615137754E63 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*2.195175807568877E63 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.390351615137754E63 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*2.195175807568877E63)*4.0E-62;

	J_JT[3 - 1][5 - 1] = cos(q4)*-2.670848298949963E4 + cos(q4)*cos(q5)*6.033374388745788E4 - sin(q4)*sin(q5)*5.479070391215557E4 + pow(cos(q2), 2.0)*cos(q4)*1.0E-28 + pow(cos(q3), 2.0)*cos(q4)*1.0E-28 + cos(q4)*pow(cos(q5), 2.0)*1.0E-28 - pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q4)*2.0E-28 - pow(cos(q2), 2.0)*cos(q4)*pow(cos(q5), 2.0)*2.0E-28 - pow(cos(q3), 2.0)*cos(q4)*pow(cos(q5), 2.0)*2.0E-28 - cos(q2)*sin(q2)*sin(q5)*2.0E-28 + cos(q3)*sin(q3)*sin(q5)*2.0E-28 - cos(q5)*sin(q4)*sin(q5)*1.0E-28 + cos(q2)*pow(cos(q3), 2.0)*sin(q2)*sin(q5)*4.0E-28 + cos(q2)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*1.0E-28 - pow(cos(q2), 2.0)*cos(q3)*sin(q3)*sin(q5)*4.0E-28 - cos(q3)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*1.0E-28 + pow(cos(q2), 2.0)*cos(q5)*sin(q4)*sin(q5)*2.0E-28 + pow(cos(q3), 2.0)*cos(q5)*sin(q4)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q4)*pow(cos(q5), 2.0)*4.0E-28 + cos(q2)*cos(q5)*sin(q2)*sin(q5)*6.0E-29 - cos(q3)*cos(q5)*sin(q3)*sin(q5)*6.0E-29 - cos(q2)*pow(cos(q3), 2.0)*cos(q5)*sin(q2)*sin(q5)*1.2E-28 + cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*4.0E-29 + pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q3)*sin(q5)*1.2E-28 - cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*4.0E-29 - cos(q2)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*cos(q3)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*2.0E-28 - pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q5)*sin(q4)*sin(q5)*4.0E-28 - cos(q2)*cos(q3)*cos(q4)*sin(q2)*sin(q3)*1.0E-28 - cos(q2)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*8.0E-29 + pow(cos(q2), 2.0)*cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*8.0E-29 + cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*2.0E-28 - cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0E-28;

	J_JT[4 - 1][1 - 1] = cos(q2)*cos(q3)*5.705418679635367E4 - cos(q2)*sin(q4)*6.638188324412958E4 + sin(q2)*sin(q3)*5.705418679635367E4 - cos(q2)*cos(q3)*pow(cos(q5), 2.0)*2.313022476545122E4 - pow(cos(q5), 2.0)*sin(q2)*sin(q3)*2.313022476545122E4 - cos(q2)*cos(q3)*cos(q4)*1.586688916567E3 + cos(q2)*cos(q3)*cos(q5)*2.843315203090245E4 - cos(q2)*cos(q4)*sin(q5)*7.200176648825917E4 - cos(q2)*cos(q5)*sin(q4)*3.600088324412958E4 + cos(q2)*sin(q3)*sin(q4)*5.562457336475938E4 - cos(q3)*sin(q2)*sin(q4)*5.562457336475938E4 - cos(q4)*sin(q2)*sin(q3)*1.586688916567E3 + cos(q5)*sin(q2)*sin(q3)*2.843315203090245E4 - cos(q2)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*1.335424149474981E4 + cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*1.335424149474981E4 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*8.605089165669998E2 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.721017833134E3 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*6.033374388745788E4 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*5.543039975302317E3 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*6.033374388745788E4 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*5.543039975302317E3 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*8.605089165669998E2 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.721017833134E3 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.670848298949963E4 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.670848298949963E4;

	J_JT[4 - 1][2 - 1] = sin(q1)*(cos(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - cos(q1)*(sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1))*1.0E-31;

	J_JT[4 - 1][3 - 1] = sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(sin(q2 - q3)*-3.435587236306403E2 + cos(q2 - q3)*sin(q4)*1.619070323027551E2 + sin(q2 - q3)*cos(q5)*1.520862412102134E2 + cos(q2 - q3)*cos(q4)*sin(q5)*1.756140646055102E2 + cos(q2 - q3)*cos(q5)*sin(q4)*8.780703230275508E1)*-1.0E-31 + sin(q1)*(cos(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) + cos(q4)*sin(q1)*sin(q2)*sin(q3))*8.780703230275508E1 - sin(q5)*(-cos(q1)*cos(q4) + cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4))*1.756140646055102E2 + cos(q2 - q3)*cos(q4)*sin(q1)*1.619070323027551E2)*(cos(q2)*cos(q3)*8.588968090766008E63 + sin(q2)*sin(q3)*8.588968090766008E63 - cos(q2)*cos(q3)*cos(q5)*3.802156030255336E63 - cos(q2)*sin(q3)*sin(q4)*4.047675807568877E63 + cos(q3)*sin(q2)*sin(q4)*4.047675807568877E63 - cos(q5)*sin(q2)*sin(q3)*3.802156030255336E63 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.390351615137754E63 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*2.195175807568877E63 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.390351615137754E63 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*2.195175807568877E63)*4.0E-62 - cos(q1)*(sin(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4))*1.756140646055102E2 - cos(q5)*(sin(q1)*sin(q4)*-1.0 + cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3))*8.780703230275508E1 - cos(q2 - q3)*cos(q1)*cos(q4)*1.619070323027551E2)*(cos(q2)*cos(q3)*8.588968090766008E63 + sin(q2)*sin(q3)*8.588968090766008E63 - cos(q2)*cos(q3)*cos(q5)*3.802156030255336E63 - cos(q2)*sin(q3)*sin(q4)*4.047675807568877E63 + cos(q3)*sin(q2)*sin(q4)*4.047675807568877E63 - cos(q5)*sin(q2)*sin(q3)*3.802156030255336E63 - cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.390351615137754E63 - cos(q2)*cos(q5)*sin(q3)*sin(q4)*2.195175807568877E63 + cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.390351615137754E63 + cos(q3)*cos(q5)*sin(q2)*sin(q4)*2.195175807568877E63)*4.0E-62;

	J_JT[4 - 1][4 - 1] = pow(sin(q2 - q3), 2.0)*pow(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33, 2.0)*1.0E-62 + pow(sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2, 2.0) + pow(cos(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2, 2.0);

	J_JT[4 - 1][5 - 1] = (cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2)*(sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2) + (sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2 + cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2)*(cos(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2) + sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(cos(q2 - q3)*sin(q5)*1.520862412102134E2 + sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 - sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1)*1.0E-31;
	
	J_JT[5 - 1][1 - 1] = cos(q2)*cos(q3)*1.542014984363415E4 + sin(q2)*sin(q3)*1.542014984363415E4 + cos(q2)*cos(q3)*cos(q5)*2.843315203090245E4 - cos(q2)*cos(q4)*sin(q5)*3.600088324412958E4 - cos(q2)*cos(q5)*sin(q4)*7.200176648825917E4 - cos(q2)*sin(q3)*sin(q4)*2.670848298949963E4 + cos(q3)*sin(q2)*sin(q4)*2.670848298949963E4 - cos(q2)*sin(q3)*sin(q5)*1.490445163860092E3 + cos(q3)*sin(q2)*sin(q5)*1.490445163860092E3 + cos(q5)*sin(q2)*sin(q3)*2.843315203090245E4 - cos(q2)*pow(cos(q5), 2.0)*sin(q3)*sin(q4)*1.0E-28 + cos(q3)*pow(cos(q5), 2.0)*sin(q2)*sin(q4)*1.0E-28 - cos(q2)*cos(q3)*cos(q4)*cos(q5)*1.721017833134E3 + cos(q2)*cos(q3)*sin(q4)*sin(q5)*8.605089165669998E2 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*5.479070391215557E4 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*6.033374388745788E4 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*5.479070391215557E4 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*6.033374388745788E4 - cos(q4)*cos(q5)*sin(q2)*sin(q3)*1.721017833134E3 + sin(q2)*sin(q3)*sin(q4)*sin(q5)*8.605089165669998E2 - cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.659964638814754E-29 + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.659964638814754E-29;

	J_JT[5 - 1][2 - 1] = -(cos(q2 - q3)*sin(q5)*1.520862412102134E2 + sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 - sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1)*(sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1)) + sin(q1)*(sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2 + cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31 - cos(q1)*(cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*2.0E-31;

	J_JT[5 - 1][3 - 1] = cos(q4)*-2.670848298949963E4 + cos(q4)*cos(q5)*6.033374388745788E4 - sin(q4)*sin(q5)*5.479070391215557E4 + pow(cos(q2), 2.0)*cos(q4)*1.0E-28 + pow(cos(q3), 2.0)*cos(q4)*1.0E-28 + cos(q4)*pow(cos(q5), 2.0)*1.0E-28 - pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q4)*2.0E-28 - pow(cos(q2), 2.0)*cos(q4)*pow(cos(q5), 2.0)*2.0E-28 - pow(cos(q3), 2.0)*cos(q4)*pow(cos(q5), 2.0)*2.0E-28 - cos(q2)*sin(q2)*sin(q5)*2.0E-28 + cos(q3)*sin(q3)*sin(q5)*2.0E-28 - cos(q5)*sin(q4)*sin(q5)*1.0E-28 + cos(q2)*pow(cos(q3), 2.0)*sin(q2)*sin(q5)*4.0E-28 + cos(q2)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*1.0E-28 - pow(cos(q2), 2.0)*cos(q3)*sin(q3)*sin(q5)*4.0E-28 - cos(q3)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*1.0E-28 + pow(cos(q2), 2.0)*cos(q5)*sin(q4)*sin(q5)*2.0E-28 + pow(cos(q3), 2.0)*cos(q5)*sin(q4)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q4)*pow(cos(q5), 2.0)*4.0E-28 + cos(q2)*cos(q5)*sin(q2)*sin(q5)*6.0E-29 - cos(q3)*cos(q5)*sin(q3)*sin(q5)*6.0E-29 - cos(q2)*pow(cos(q3), 2.0)*cos(q5)*sin(q2)*sin(q5)*1.2E-28 + cos(q2)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*4.0E-29 + pow(cos(q2), 2.0)*cos(q3)*cos(q5)*sin(q3)*sin(q5)*1.2E-28 - cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*4.0E-29 - cos(q2)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*sin(q2)*sin(q5)*2.0E-28 + pow(cos(q2), 2.0)*cos(q3)*pow(cos(q4), 2.0)*sin(q3)*sin(q5)*2.0E-28 - pow(cos(q2), 2.0)*pow(cos(q3), 2.0)*cos(q5)*sin(q4)*sin(q5)*4.0E-28 - cos(q2)*cos(q3)*cos(q4)*sin(q2)*sin(q3)*1.0E-28 - cos(q2)*pow(cos(q3), 2.0)*pow(cos(q4), 2.0)*cos(q5)*sin(q2)*sin(q5)*8.0E-29 + pow(cos(q2), 2.0)*cos(q3)*pow(cos(q4), 2.0)*cos(q5)*sin(q3)*sin(q5)*8.0E-29 + cos(q2)*cos(q3)*cos(q4)*pow(cos(q5), 2.0)*sin(q2)*sin(q3)*2.0E-28 - cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0E-28;

	J_JT[5 - 1][4 - 1] = (cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2)*(sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2) + (sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2 + cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2)*(cos(q1)*sin(q4)*1.619070323027551E2 + sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 + cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2) + sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*(cos(q2 - q3)*sin(q5)*1.520862412102134E2 + sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 - sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1)*1.0E-31;

	J_JT[5 - 1][5 - 1] = pow(sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2 + cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2, 2.0) + pow(cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2, 2.0) + pow(cos(q2 - q3)*sin(q5)*1.520862412102134E2 + sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 - sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1, 2.0);

	/*	Convert JJ^T to a float, or double, or whichever is needed for testing	*/	
	double JJT_F[5][5];
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			JJT_F[i][j] = (double)(J_JT[i][j]);
		}
	}

	printf("%e\n", JJT_F[0][0]);
	printf("%e\n", JJT_F[0][1]);
	printf("%e\n", JJT_F[0][2]);
	printf("%e\n", JJT_F[0][3]);
	printf("%e\n", JJT_F[0][4]);
	printf("%e\n", JJT_F[1][0]);
	printf("%e\n", JJT_F[1][1]);
	printf("%e\n", JJT_F[1][2]);
	printf("%e\n", JJT_F[1][3]);
	printf("%e\n", JJT_F[1][4]);
	printf("%e\n", JJT_F[2][0]);
	printf("%e\n", JJT_F[2][1]);
	printf("%e\n", JJT_F[2][2]);
	printf("%e\n", JJT_F[2][3]);
	printf("%e\n", JJT_F[2][4]);
	printf("%e\n", JJT_F[3][0]);
	printf("%e\n", JJT_F[3][1]);
	printf("%e\n", JJT_F[3][2]);
	printf("%e\n", JJT_F[3][3]);
	printf("%e\n", JJT_F[3][4]);
	printf("%e\n", JJT_F[4][0]);
	printf("%e\n", JJT_F[4][1]);
	printf("%e\n", JJT_F[4][2]);
	printf("%e\n", JJT_F[4][3]);
	printf("%e\n", JJT_F[4][4]);


	/*	now need to invert J*J^T	*/

	/*	Calculate the determinant by LU decomposition, and taking the straight determinant	*/
	double d = determinant(JJT_F, 5);
	double dd = LU_decomposition(JJT_F, 5);

	printf("determinant: %f    det(LU): %f\n", d, dd);

	printf("\n\n    * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n\n\tDeterminant of the Matrix = %f\n", d);
	
	/*	Ensure the determinant exists	*/
	if (d == 0.0E-50)
	{
		printf("\n\n    * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n\n\tInverse not exist\n\n");

	}
	else { 
		/*	Invert JJ^T as the determinant exists	*/
		double m_inverse[25][25];

		cofactor(JJT_F, m_inverse, 5, d);
	
		/*	Define J^T, so the calculation J^T(JJ^T)^-1 can be computed	*/
		double sum = 0;
		double transJ[5][5];

		transJ[0][0] = cos(q1)*9.8 - cos(q1)*cos(q4)*1.619070323027551E2 - cos(q2)*sin(q1)*4.1E2 + cos(q2 - q3)*sin(q1)*sin(q4)*1.619070323027551E2 - cos(q1)*cos(q4)*cos(q5)*8.780703230275508E1 + cos(q2)*sin(q1)*sin(q3)*3.435587236306403E2 - cos(q3)*sin(q1)*sin(q2)*3.435587236306403E2 + cos(q1)*sin(q4)*sin(q5)*1.756140646055102E2 - cos(q2)*cos(q5)*sin(q1)*sin(q3)*1.520862412102134E2 + cos(q3)*cos(q5)*sin(q1)*sin(q2)*1.520862412102134E2 + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*1.756140646055102E2 + cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*8.780703230275508E1 + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*1.756140646055102E2 + cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*8.780703230275508E1;
		transJ[0][1] = sin(q1)*9.8 + cos(q1)*cos(q2)*4.1E2 - cos(q4)*sin(q1)*1.619070323027551E2 + sin(q1)*sin(q4)*sin(q5)*1.756140646055102E2 - cos(q2 - q3)*cos(q1)*sin(q4)*1.619070323027551E2 - cos(q1)*cos(q2)*sin(q3)*3.435587236306403E2 + cos(q1)*cos(q3)*sin(q2)*3.435587236306403E2 - cos(q4)*cos(q5)*sin(q1)*8.780703230275508E1 + cos(q1)*cos(q2)*cos(q5)*sin(q3)*1.520862412102134E2 - cos(q1)*cos(q3)*cos(q5)*sin(q2)*1.520862412102134E2 - cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*1.756140646055102E2 - cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4)*8.780703230275508E1 - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*1.756140646055102E2 - cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*8.780703230275508E1;
		transJ[1][0] = cos(q1)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*(-2.0E-31);
		transJ[1][1] = sin(q1)*(sin(q2)*2.05E33 - cos(q2)*cos(q3)*1.717793618153202E33 - sin(q2)*sin(q3)*1.717793618153202E33 + cos(q2)*cos(q3)*cos(q5)*7.604312060510672E32 + cos(q2)*sin(q3)*sin(q4)*8.095351615137753E32 - cos(q3)*sin(q2)*sin(q4)*8.095351615137753E32 + cos(q5)*sin(q2)*sin(q3)*7.604312060510672E32 + cos(q2)*cos(q4)*sin(q3)*sin(q5)*8.780703230275508E32 + cos(q2)*cos(q5)*sin(q3)*sin(q4)*4.390351615137754E32 - cos(q3)*cos(q4)*sin(q2)*sin(q5)*8.780703230275508E32 - cos(q3)*cos(q5)*sin(q2)*sin(q4)*4.390351615137754E32)*(-2.0E-31);
		transJ[1][2] = sin(q2 - q3)*3.435587236306403E2 + cos(q2)*4.1E2 - cos(q2 - q3)*sin(q4 + q5)*8.780703230275508E1 - cos(q2 - q3)*sin(q4)*1.619070323027551E2 + cos(q2 - q3)*sin(q4 - q5)*8.780703230275508E1 - cos(q5)*(sin(q2 - q3)*1.520862412102134E2 + cos(q2 - q3)*sin(q4)*8.780703230275508E1);
		transJ[2][0] = cos(q5)*(sin(q4)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*5.0E-1 + cos(q1)*cos(q2)*cos(q3)*8.660254037844386E-1 + cos(q1)*sin(q2)*sin(q3)*8.660254037844386E-1)*1.756140646055102E2 + sin(q4)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*1.619070323027551E2 + cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2))*1.756140646055102E2 - cos(q1)*cos(q2)*cos(q3)*3.435587236306403E2 - cos(q1)*sin(q2)*sin(q3)*3.435587236306403E2;
		transJ[2][1] = cos(q5)*(sin(q4)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*5.0E-1 + sin(q1)*sin(q2)*sin(q3)*8.660254037844386E-1 + cos(q2)*cos(q3)*sin(q1)*8.660254037844386E-1)*1.756140646055102E2 + sin(q4)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*1.619070323027551E2 - sin(q1)*sin(q2)*sin(q3)*3.435587236306403E2 + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2))*1.756140646055102E2 - cos(q2)*cos(q3)*sin(q1)*3.435587236306403E2;
		transJ[2][2] = sin(q2 - q3)*-3.435587236306403E2 + cos(q2 - q3)*sin(q4)*1.619070323027551E2 + sin(q2 - q3)*cos(q5)*1.520862412102134E2 + cos(q2 - q3)*cos(q4)*sin(q5)*1.756140646055102E2 + cos(q2 - q3)*cos(q5)*sin(q4)*8.780703230275508E1;
		transJ[3][0] = sin(q1)*sin(q4)*1.619070323027551E2 + cos(q5)*(sin(q1)*sin(q4)*5.0E-1 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1)*1.756140646055102E2 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*1.619070323027551E2 + sin(q5)*(cos(q4)*sin(q1)*1.0 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2;
		transJ[3][1] = cos(q1)*sin(q4)*-1.619070323027551E2 - sin(q5)*(cos(q1)*cos(q4)*1.0 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2 - cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*1.619070323027551E2 - cos(q5)*(cos(q1)*sin(q4)*5.0E-1 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1)*1.756140646055102E2;
		transJ[3][2] = sin(q2 - q3)*(cos(q4)*1.619070323027551E33 + cos(q4)*cos(q5)*8.780703230275508E32 - sin(q4)*sin(q5)*1.756140646055102E33)*-1.0E-31;
		transJ[4][0] = cos(q5)*(sin(q1)*sin(q4)*1.0 - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)))*1.756140646055102E2 + sin(q5)*(cos(q4)*sin(q1)*5.0E-1 + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))*5.0E-1 - cos(q1)*cos(q2)*sin(q3)*8.660254037844386E-1 + cos(q1)*cos(q3)*sin(q2)*8.660254037844386E-1)*1.756140646055102E2;
		transJ[4][1] = sin(q5)*(cos(q1)*cos(q4)*5.0E-1 - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))*5.0E-1 + cos(q2)*sin(q1)*sin(q3)*8.660254037844386E-1 - cos(q3)*sin(q1)*sin(q2)*8.660254037844386E-1)*-1.756140646055102E2 - cos(q5)*(cos(q1)*sin(q4)*1.0 + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))*1.756140646055102E2;
		transJ[4][2] = cos(q2 - q3)*sin(q5)*-1.520862412102134E2 - sin(q2 - q3)*cos(q4)*cos(q5)*1.756140646055102E2 + sin(q2 - q3)*sin(q4)*sin(q5)*8.780703230275508E1;

		/*	Calculate J inverse	*/
		double J_inv[25][25];

		for (int c = 0; c < 5; c++) {
			for (int j = 0; j < 5; j++) {
				for (int k = 0; k < 5; k++) {
					sum = sum + (double)(transJ[c][k]) * m_inverse[k][j];
				}

				J_inv[c][j] = sum;
				sum = 0;
			}
		}
	
		/*	print out the elements of J inverse	*/
		for (int c = 0; c < 5; c++) {
			for (int j = 0; j < 5; j++)
				printf("%f\t", J_inv[c][j]);

			printf("\n");
		}
	

	}

}


/*	Uses the LU decomposition to split A into diagonal matrices
*	calculate the determinant and returns as a double	*/
double LU_decomposition(double A[10][10], int size) {

	double L[10][10];
	double U[10][10];

	for (int j = 0; j<size; j++) {
		for (int i = 0; i<size; i++) {
			if (i <= j) {
				U[i][j] = A[i][j];
				for (int k = 0; k <= i - 1; k++) {
					U[i][j] -= L[i][k] * U[k][j];
				}
				
				if (i == j){
					L[i][j] = 1;
				}
				else {
					L[i][j] = 0;

				}
			}
			else
			{
				L[i][j] = A[i][j];
				
				for (int k = 0; k <= j - 1; k++) {
					L[i][j] -= L[i][k] * U[k][j];
				}
				
				L[i][j] /= U[j][j];
				U[i][j] = 0;
			}
		}
	}

	/*	print the results*/
	printf("[L]: \n");
	for (int i = 0; i<size; i++)
	{
		for (int j = 0; j<size; j++)
			printf(" %e ", L[i][j]);
		printf("\n");
	}
	printf("\n\n[U]: \n");
	for (int i = 0; i<size; i++)
	{
		for (int j = 0; j<size; j++)
			printf(" %e ", U[i][j]);
		printf("\n");
	}

	/*	Calculate the determinant from the LU decomposition matrices	*/
	double detL = 0;
	double detU = 0;

	/*	det(diagonal matrix) = prod(diagonal elements)	*/
	for (int j = 0; j < size; j++) {
		if (j == 0) {
			detL = L[j][j];
			detU = U[j][j];
		}
		else {
			detL = detL * L[j][j];
			detU = detU * U[j][j];
		}
	}

	printf("Det: %e %e %e\n", detL, detU, detL * detU);
	return detL * detU;
}


/* For calculating Determinant of the Matrix . this function is recursive */
double determinant(double matrix[25][25], double size)
{

	double s = 1, det = 0, m_minor[25][25];
	int i, j, m, n, c;
	if (size == 1)
	{

		return (matrix[0][0]);
	}
	else
	{
		det = 0.0;
		for (c = 0; c<size; c++)
		{
			m = 0;
			n = 0;
			for (i = 0; i<size; i++)
			{
				for (j = 0; j<size; j++)
				{
					m_minor[i][j] = 0;
					if (i != 0 && j != c)
					{
						m_minor[m][n] = matrix[i][j];
						if (n<(size - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (matrix[0][c] * determinant(m_minor, size - 1));
			s = -1 * s;
		}
	}

	return (det);
}

/*calculate cofactor of matrix*/
void cofactor(double matrix[25][25], double **invs, double size, double det) {
	double m_cofactor[25][25], matrix_cofactor[25][25];
	int p, q, m, n, i, j;
	for (q = 0; q<size; q++)
	{
		for (p = 0; p<size; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i<size; i++)
			{
				for (j = 0; j<size; j++)
				{
					if (i != q && j != p)
					{
						m_cofactor[m][n] = matrix[i][j];
						if (n<(size - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			matrix_cofactor[q][p] = powf(-1.0, q + p) * determinant(m_cofactor, size - 1);
		}
	}
	transpose(matrix, invs, matrix_cofactor, size, det);
}


/*Finding transpose of cofactor of matrix*/
void transpose(double matrix[25][25], double **invs, double matrix_cofactor[25][25], double size, double det)
{
	int i, j;
	double m_transpose[25][25], m_inverse[25][25], d;

	for (i = 0; i<size; i++)
	{
		for (j = 0; j<size; j++)
		{
			m_transpose[i][j] = matrix_cofactor[j][i];
		}
	}

	d = det;

	printf("\n#%e\n", d);

	for (i = 0; i<size; i++)
	{
		for (j = 0; j<size; j++)
		{
			m_inverse[i][j] = m_transpose[i][j] / d;
		}
	}
	printf("\n\n\t* * * * * * * * * * * * * * * * * * * * * * * \n\n\tThe inverse of matrix is : \n\n");

	for (i = 0; i<size; i++)
	{
		for (j = 0; j<size; j++)
		{
			printf(" %e", m_inverse[i][j]);
			invs[i][j] = m_inverse[i][j];
		}
		printf("\n\n");
	}
	printf("\n\n* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *");
	printf("\n* * * * * * * * * * * * * * * * * THE END * * * * * * * * * * * * * * * * * * *");
	printf("\n* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *");

}


/*	Depreciated IK function for a 3R arm	*/
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

	cos_tha = (px*(a + b * cos(th_b)) + py * b*sin(th_b)) / delta;
	sin_tha = (py*(a + b * cos(th_b)) - px * b*sin(th_b)) / delta;

	th_a = atan2(sin_tha, cos_tha);

	k = ((px*px) + (py*py) - (a*a) - (b*b)) / (2.0 * a*b);
	th_b = -1.0*acos(pow((k*k), 0.5));

	th_c = acos((px - a * cos(th_a) - b * cos(th_a + th_b)) / c) - th_a - th_b;

}
