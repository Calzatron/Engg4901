/*
*	The kinematics library for the Jaco arm in VREP	
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

move* move_ptr;
void define_classic_parameters(move* move_ptr);
void fk_classic(move* move_ptr, info* info_ptr);
void fk_mod(move* move_ptr);
int forward_xy_a(move* move_ptr);
void ik_RRR_arm(move* move_ptr, char* plane);



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
	move_ptr->d_i[4] = -(move_ptr->lengthD[3] + move_ptr->lengthD[4] * aa);
	move_ptr->d_i[5] = -(move_ptr->lengthD[4] * aa + move_ptr->lengthD[5] * aa);
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
	float position[3];
	get_position_vrep(info_ptr, &position, 33);
	//simxGetObjectPosition(info_ptr->clientID, 33, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_position_vrep(info_ptr, &position, 30);
	//simxGetObjectPosition(info_ptr->clientID, 30, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_position_vrep(info_ptr, &position, 27);
	//simxGetObjectPosition(info_ptr->clientID, 27, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_position_vrep(info_ptr, &position, 24);
	//simxGetObjectPosition(info_ptr->clientID, 24, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_position_vrep(info_ptr, &position, 21);
	//simxGetObjectPosition(info_ptr->clientID, 21, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_position_vrep(info_ptr, &position, 18);
	//simxGetObjectPosition(info_ptr->clientID, 18, 18, &position, simx_opmode_blocking);
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

	cos_tha = (px*(a + b * cos(th_b)) + py * b*sin(th_b)) / delta;
	sin_tha = (py*(a + b * cos(th_b)) - px * b*sin(th_b)) / delta;

	th_a = atan2(sin_tha, cos_tha);

	k = ((px*px) + (py*py) - (a*a) - (b*b)) / (2.0 * a*b);
	th_b = -1.0*acos(pow((k*k), 0.5));

	th_c = acos((px - a * cos(th_a) - b * cos(th_a + th_b)) / c) - th_a - th_b;

}



int forward_xy_a(move* move_ptr) {

	double c;

	double ang = (3.141592 / 2.0) - move_ptr->currAng[2];
	double H = c / cos(move_ptr->currAng[3]);


	return 0;

}