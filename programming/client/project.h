
#ifndef PROJECT_H_
#define PROJECT_H_

#define DEBUG	1

/*	Define motion struct	*/





typedef struct info {

	int clientID;						/*	ID for networking	*/
	int objectCount;					/*	Number of objects in the scene	*/
	int* objectHandles;					/*	The ID corresponding to the i'th object	*/
	int* isJoint;						/*	Boolean array, for i'th handle as a joint	*/
	char** objectNames;					/*	Array of names of objects in scene	*/
	int* jacoArmJointHandles;			/*	joint ID array	*/
	char* response;

	int targetHandle;

	char* sceneMode;

	float* armPosition;

	char* programMode;

} info;




typedef struct move {
	double* alpha;
	double* a_i;
	double* d_i;
	double* lengthD;
	double currPos[3];
	double lastPos[3];
	double currAng[6];
	double nextAng[6];
	double motorAng[6];

} move;


void get_position_vrep(info* info_ptr, float* position, int handle);
void set_world_position_vrep(info* info_ptr, float* position, int objectHandle);
void get_world_position_vrep(info* info_ptr, float* position, int handle);
void move_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void set_joint_angle_vrep(info* info_ptr, move* move_ptr, int jointNum, double ang);
void pause_communication_vrep(info* info_ptr, int status);

#endif // !PROJECT.H
