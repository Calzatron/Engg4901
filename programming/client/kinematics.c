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
int inverse_kinematics(move* move_ptr, info* info_ptr, float* position, double* angles);
void control_kinematics(info* info_ptr, move* move_ptr, float x, float y, float z);


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


void fk_classic(move* move_ptr, info* info_ptr) {

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
	//q1 = -q_1;
	//q1 = q1 + pi;
	q2 = -(pi / 2.0) + (q_2 + pi);
	q3 = (pi / 2.0) + (q_3 + pi);
	q4 = q_4;
	q5 = -pi + q_5;
	q6 = 0;
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


/*	Calculates the position of the tip using classic DH parameters
*	ret = T * pt
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

	/*	changed to world position, should be of base, different function that's not extern yet	*/

	get_world_position_vrep(info_ptr, &position, 33);
	//simxGetObjectPosition(info_ptr->clientID, 33, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_world_position_vrep(info_ptr, &position, 30);
	//simxGetObjectPosition(info_ptr->clientID, 30, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_world_position_vrep(info_ptr, &position, 27);
	//simxGetObjectPosition(info_ptr->clientID, 27, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_world_position_vrep(info_ptr, &position, 24);
	//simxGetObjectPosition(info_ptr->clientID, 24, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_world_position_vrep(info_ptr, &position, 21);
	//simxGetObjectPosition(info_ptr->clientID, 21, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);
	get_world_position_vrep(info_ptr, &position, 18);
	//simxGetObjectPosition(info_ptr->clientID, 18, 18, &position, simx_opmode_blocking);
	printf("%f %f %f\n", position[0], position[1], position[2]);

}


/*	Uses the position of the fourth joint to calculate angles q_2 and q_3
*	After, the position of S' is calculated (position of the tip for the given
*	and current angles)
*	The target is then moved to S' for the arm to follow
*/
int inverse_kinematics(move* move_ptr, info* info_ptr, float* position, double* angles) {
	
	double pi = 3.141594;
	//float position[4];// = malloc(sizeof(float) * 3);
	//position[0] = 0; position[1] = 0; position[2] = 0; position[3] = 0;

	//float basePosition[4]; basePosition[0] = 0; basePosition[1] = 0; basePosition[2] = 0; basePosition[3] = 0;
	//get_world_position_vrep(info_ptr, basePosition, baseHandle);
	//basePosition[0] = info_ptr->armPosition[0];
	//basePosition[1] = info_ptr->armPosition[1];
	//basePosition[2] = info_ptr->armPosition[2];
	//printf("Base at %f %f %f\n", basePosition[0], basePosition[1], basePosition[2]);
	//int joint4Handle = 27;

	//get_world_position_vrep(info_ptr, position, joint4Handle);
	#ifdef DEBUG
		printf("Tip desired: %f %f %f\n", position[0], position[1], position[2]);
	#endif // DEBUG
			
	//double px = (double)((position[0] - basePosition[0])*1000.0);
	//double py = (double)((position[1] - basePosition[1])*1000.0);
	double px = (double)((position[0])*1000.0);
	double py = (double)((position[1])*1000.0);
	double pz = (double)(position[2])*1000.0;
	double d1 = 197.13;
	double d2 = 410.0;
	double d3 = 207.3;
	double e2 = 9.8;

	double j = pow(px, 2) + pow((pz - d1), 2) - (d2*d2) - (d3*d3);
	double k = 2.0 * d2 * d3;

	if (fabsf(j) > fabsf(k)) {
		#ifdef DEBUG
		printf("x,z,i,j:	%f %f %f %f\n", px, pz, j, k);
		#endif // DEBUG

		return 1;
	}

	double q_3 = 1 * acos( j / k );

	printf("i,j,q_3:	%f %f %f\n", j, k, q_3);

	double cq_2 = (px*(d2 + d3 * cos(q_3)) + d3 * sin(q_3)*(pz - d1)) / (pow(d2,2) + pow(d3,2) + 2 * d2*d3*cos(q_3));
	double sq_2 = (-px * d3*sin(q_3) + (d2 + d3 * cos(q_3))*(pz - d1)) / (pow(d2, 2) + pow(d3, 2) + 2 * d2*d3*cos(q_3));
	double q_2 = (pi / 2) - atan2(sq_2, cq_2);

	printf("c,s,q_2:	%f %f %f\n", cq_2, sq_2, q_2);

	if (((move_ptr->currAng[3 - 1] - pi / 4 < q_3) && (move_ptr->currAng[3 - 1] + pi / 4 > q_3)) ||
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

	//Instead of calculating q_1 should read from vrep
	//double q_1 = atan2f(py, px) - asin((e2) / (pow(pow(px, 2) + pow(py, 2), .5)));
	//double q_1 = tan(py / px) - sin((100.0 + e2) / (pow(pow(px,2) + pow(py,2), .5)));
	double q_1 = move_ptr->currAng[1 - 1];
	double q_4 = move_ptr->currAng[4 - 1];
	double q_5 = move_ptr->currAng[5 - 1];
	#ifdef DEBUG
		printf("Angles are:		%f %f %f %f %f\n", q_1, q_2, q_3, q_4, q_5);
	#endif // DEBUG

	


	/*	Transform angles into DH	*/
	double q1, q2, q3, q4, q5, q6;
	q1 = -pi - q_1;
	//q1 = -q_1;
	//q1 = q1 + pi;
	q2 = -(pi / 2.0) + (q_2 + pi);
	q3 = (pi / 2.0) + (q_3 + pi);
	q4 = q_4;
	q5 = -pi + q_5;
	q6 = 0;
	double T[3];
	
	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;

//	if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
//	#ifdef DEBUG
//		printf("GGG");
//		
//
//	#endif // 
//		
//	}
//	else {
//#		ifdef DEBUG
//			printf("HHH");
//		#endif // DEBUG
//		//	exit(1);
//		//return 2;
//	}
	//else {
	//	q1 = - q_1;
	//	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	//	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	//	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;
	//}

	//if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
	//	printf("HHH");
	//}
	//else {
	//	q1 = -q_1;
	//	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	//	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	//	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;
	//}

	//if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
	//	printf("JJJ");
	//}
	//else {
	//	q1 = q_1;
	//	T[1 - 1] = 9.8*sin(q1) + 410.0*cos(q1)*cos(q2) - 161.90703230275506147226218323136*cos(q4)*sin(q1) + 175.614064605510166451876962007*sin(q5)*(1.0*sin(q1)*sin(q4) - cos(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) - 161.90703230275506147226218323136*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 175.614064605510166451876962007*cos(q5)*(0.5*cos(q4)*sin(q1) + 0.5*sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3)) - 0.86602540378443864676372317075294*cos(q1)*cos(q2)*sin(q3) + 0.86602540378443864676372317075294*cos(q1)*cos(q3)*sin(q2)) - 343.55872363064032981583295622841*cos(q1)*cos(q2)*sin(q3) + 343.55872363064032981583295622841*cos(q1)*cos(q3)*sin(q2);
	//	T[2 - 1] = 161.90703230275506147226218323136*cos(q1)*cos(q4) - 9.8*cos(q1) + 410.0*cos(q2)*sin(q1) + 175.614064605510166451876962007*cos(q5)*(0.5*cos(q1)*cos(q4) - 0.5*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.86602540378443864676372317075294*cos(q2)*sin(q1)*sin(q3) - 0.86602540378443864676372317075294*cos(q3)*sin(q1)*sin(q2)) - 175.614064605510166451876962007*sin(q5)*(1.0*cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) - 161.90703230275506147226218323136*sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 343.55872363064032981583295622841*cos(q2)*sin(q1)*sin(q3) + 343.55872363064032981583295622841*cos(q3)*sin(q1)*sin(q2);
	//	T[3 - 1] = 410.0*sin(q2) - 343.55872363064032981583295622841*cos(q2)*cos(q3) - 343.55872363064032981583295622841*sin(q2)*sin(q3) + 175.614064605510166451876962007*cos(q5)*(0.86602540378443864676372317075294*cos(q2)*cos(q3) + 0.86602540378443864676372317075294*sin(q2)*sin(q3) + 0.5*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2))) + 161.90703230275506147226218323136*sin(q4)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 175.614064605510166451876962007*cos(q4)*sin(q5)*(1.0*cos(q2)*sin(q3) - 1.0*cos(q3)*sin(q2)) + 197.13;
	//}
	//if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
	//	// same sign -> same coords
	//	printf("KKK");
	//}

	angles[0] = (q_1);
	angles[1] = (q_2);
	angles[2] = (q_3);
	angles[3] = (q_4);
	angles[4] = (q_5);
	angles[5] = (0.0);

	T[0] = (T[0] / 1000.0);
	T[1] = (T[1] / 1000.0);
	T[2] = (T[2] / 1000.0);
	#ifdef DEBUG
		printf("Calc position of Tip at:		%f %f %f\n", T[0], T[1], T[2]);
	#endif // DEBUG

	//get_world_position_vrep(info_ptr, &position, info_ptr->targetHandle-1);
	//printf("Position of Tip at:			%f %f %f\n", position[0], position[1], position[2]);
	//int targetHandle = info_ptr->targetHandle;


	//float desTargetPos[3];
	position[0] = (float)(T[0]);
	position[1] = (float)(T[1]);
	position[2] = (float)(T[2]);

	//exit(1);
	
	if ((T[0] / position[0] > 0) && (T[1] / position[1] > 0)) {
#ifdef DEBUG
		printf("GGG");
		return 0;
#endif // 

	}
	else {
#		ifdef DEBUG
		printf("HHH");
		return 2;
#endif // DEBUG
	}

}

/*	
*	@brief calculates the desired tip position and localises the joint4 position
*	to acheive this
*	@param the information and movement structures with a change in coordinates to move
*	@ret none
*/
void control_kinematics(info* info_ptr, move* move_ptr, float x, float y, float z) {
	
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
	double initialAngles[6] = { move_ptr->currAng[0], move_ptr->currAng[1], move_ptr->currAng[2], move_ptr->currAng[3], 0.0, 0.0 };
	double angles[6] = { 0, 0, 0, 0, 0, 0 };
	int loop = 1;

	#ifdef DEBUG
		printf("calculating");
	#endif // DEBUG
			
	for(int run = 0; run < 15; run++) {

		/*	Update the desired Joint4 position	*/
		float position[4] = { J4_desired[0] + S_error[0], J4_desired[1] + S_error[1], J4_desired[2] + S_error[2], 0.0 };
		/*	update position with the calculated tip position 	*/
		int errorCheck = inverse_kinematics(move_ptr, info_ptr, position, angles);
		if (errorCheck == 1) {
			printf("IK ERROR %d\n", errorCheck);
			break;
		}
		else if (errorCheck == 2) {
			if ((S_desired[0] / position[0] < 0) || (S_desired[1] / position[1] < 0)) {
				// coordinate frame changed...
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
		if ((fabs(S_desired[0] - position[0]) < 0.02) && (fabs(S_desired[1] - position[1]) < 0.02) 
				&& (fabs(S_desired[2] - position[2]) < 0.02)) {
			loop = 0;
			break;
		}

		/*	Add some proportionality to the error to ensure limited overshoot
		*	between calculations
		*/
		S_error[0] += 0.2*(S_desired[0] - position[0]);
		S_error[1] += 0.2*(S_desired[1] - position[1]);

		if (((move_ptr->currAng[1] < 0.01) || (move_ptr->currAng[1] > 6.24)) &&
			((move_ptr->currAng[2] < 0.01) || (move_ptr->currAng[2] > 6.24))) {
			S_desired[2] = position[2];
			// arm is fully extended
			// there's no hope of reaching a higher arm position from joint4 pos
		}
		else {
			S_error[2] += 0.1*(S_desired[2] - position[2]);
		}
		printf("S_error:	%f %f %f\n", S_error[0], S_error[1], S_error[2]);
	}

	/*	Got angles, move the arm	*/
	//double jointTwoAngle = fmod(move_ptr->currAng[1] + 3.141592, 2*3.141592);
	//printf("before: %f\n", jointTwoAngle);
	//double newJointTwoAngle = fmod(angles[1] + 3.141592, 2 * 3.141592);
	//printf("after: %f\n", newJointTwoAngle);
	//
	//if ((jointTwoAngle < 3.141592) && (newJointTwoAngle > 3.141592)) {
	//	angles[1] -= 3.141592;
	//	
	//	printf("new angle: %f\n", angles[1]);
	//}
	//else if ((jointTwoAngle > 3.141592) && (newJointTwoAngle < 3.141592)) {
	//	angles[1] += 3.141592;
	//	printf("new angle: %f\n", angles[1]);
	//}
	
	//exit(1);

	pause_communication_vrep(info_ptr, 1);
	set_joint_angle_vrep(info_ptr, move_ptr, 2, angles[1]);
	set_joint_angle_vrep(info_ptr, move_ptr, 3, angles[2]);
	pause_communication_vrep(info_ptr, 0);
}



/* Updates current position vector signing modified DH parameters
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