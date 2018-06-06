/*
*	METR4901		2017-2018		UQ
*	Callum Rohweder
*	project.h
*
*	VREP client program header file
*/


#ifndef PROJECT_H_
#define PROJECT_H_


/*	Defines for debug printing	*/
//#define DEBUG	1
//#define DATA_STREAMING
//#define JOYSTICK_DEBUG


/*	Define information struct	*/
typedef struct info {

	int clientID;						/*	ID for networking	*/
	int objectCount;					/*	Number of objects in the scene	*/
	int* objectHandles;					/*	The ID corresponding to the i'th object	*/
	int* isJoint;						/*	Boolean array, for i'th handle as a joint	*/
	char** objectNames;					/*	Array of names of objects in scene	*/
	int* jacoArmJointHandles;			/*	joint ID array	*/
	char* response;						/*	Contains the latest input command	*/

	int targetHandle;					/*	Valid of the object handle for target	*/

	char* sceneMode;					/*	string specifying the Jaco arm's fk or ik state in VREP	*/

	float* armPosition;					/*	contains the world coordinates of the Jaco arm's base in VREP	*/

	char* programMode;					/*	string specifying the mode the program should operate in (ik/fk)	*/

} info;



/*	Define movement and kinematics struct	*/
typedef struct move {
	double* alpha;			// alpha D-H parameters
	double* a_i;			// a_1 D-H parameters
	double* d_i;			// d_i D-H parameters
	double* lengthD;		// link lengths of the Jaco arm
	double currPos[3];		// the current position of the end-effector
	double lastPos[3];		// the last position of the end-effector
	double currAng[6];		// the current angles of the arm
	double nextAng[6];		// the next angles to move to
	double motorAng[6];		// the angles of the motors in the fingers of the gripper

} move;



/*	external functions	*/
void get_position_vrep(info* info_ptr, float* position, int handle);
void set_world_position_vrep(info* info_ptr, float* position, int objectHandle);
void get_world_position_vrep(info* info_ptr, float* position, int handle);
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang, bool getJointAngles);
void set_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void pause_communication_vrep(info* info_ptr, int status);
double current_angle(move* move_ptr, int jointNum);



#endif // !PROJECT.H
