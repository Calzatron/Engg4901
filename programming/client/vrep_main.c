#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "extApi.h"
#include <string.h>
#include <math.h>


typedef struct info {
	
    int clientID;
    int objectCount;
    int* objectHandles;
    int* isJoint;
    char** objectNames;
	int* jacoArmJointHandles;
    char* response;

} info;

typedef struct move {
	double* alpha;
	double* a_i;
	double* d_i;
	double* lengthD;
	double currPos[3];
	double nextPos[3];
	double currAng[6];
	double nextAng[6];
	double motorAng[6];

} move;

info* makeInfo(void); 
move* makeMove(void);
void get_object_names_vrep(info* info_ptr);
void write_object_info(info* info_ptr, char* filename);
void get_command(info* info_ptr);
void get_joint_angles_vrep(info* info_ptr, move* move_ptr);
void initial_arm_config_vrep(info* info_ptr, move* move_ptr);


int main(int argc, char** argv){
    
    printf("getting client\n");
    simxFinish(-1);
    info* info_ptr = makeInfo();
	move* move_ptr = makeMove();
    info_ptr->response = malloc(sizeof(char)*128);
    //while(1){
    //}
    info_ptr->clientID = simxStart((simxChar*)"127.0.0.1",19999,true,true,5000,5);
    printf("got client\n");
    if (info_ptr->clientID != -1) {
        printf("Successfully connected to Vrep\n");
        // retrieve data in a blocking fashion - ensure response from vrep
        int objectCount;        // integer variable
        int* objectHandles;     //array of ints of unknown size
        
        int ret = simxGetObjects(info_ptr->clientID, sim_handle_all, &objectCount, 
                &objectHandles, simx_opmode_blocking);
        if (ret == simx_return_ok){
            printf("Number of objects in scene: %d \n", objectCount);
            /*  store all object handles  */;
            info_ptr->objectCount = objectCount;
            info_ptr->objectHandles = malloc(sizeof(int)*info_ptr->objectCount);
            info_ptr->isJoint = malloc(sizeof(int)*info_ptr->objectCount);
            info_ptr->objectHandles = objectHandles;
            printf(">> %d\n", info_ptr->objectHandles[40]);

            /*  retrieve all object names from custom function in VRep Main */
			int en_name = 0;
			
			if (en_name) {
				get_object_names_vrep(info_ptr);
			}
			else {
				for (int h = 0; h < info_ptr->objectCount; h++) {
					if ((h == 18) || (h == 21) || (h == 24) || (h == 27) || (h == 30) || (h == 33)) {
						info_ptr->isJoint[h] = 1;
						printf("H = %d\n", h);
					}
					else {
						info_ptr->isJoint[h] = 0;
					}

				}
			}

			/* Retrieve all angles of joints to be used in kinematics */
			initial_arm_config_vrep(info_ptr, move_ptr);

            /*  Print a sample name to check */
            //printf("aa: %s\n", info_ptr->objectNames[40]);

            /*  write information to a file */
			if (en_name) {
				if (argc == 2) {
					/* filename specified */
					write_object_info(info_ptr, argv[1]);
				}
				else if (argc == 1) {
					write_object_info(info_ptr, "objects");
				}
			}
            printf("Written to file\n");
            get_command(info_ptr);
        } else {
            printf("Remote API function call returned with error: %d\n", ret);
        }

        extApi_sleepMs(2000);

        simxFinish(-1); // close all open connections
    } else {

        printf("Connection failed %d\n", info_ptr->clientID);
        return 1;
    }
    return 0;
}

info* makeInfo(void){
    /* Initialize a struct to store all the Vrep scene info */
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}

move* makeMove(void) {
	/* Initialize a struct to store all the Vrep scene info */
	move* move_ptr = malloc(sizeof(move));
	return move_ptr;
}

void get_command(info* info_ptr){
    
    printf(">> ");
    int c;
    int i = 0;;
    
    char response[128];
    while((c = getchar()) != '\n'){
        response[i] = c;
        ++i;
    }
    response[i] = '\0';
    strcpy(info_ptr->response, response);
    printf("received: %s\n", info_ptr->response);
}


void get_object_names_vrep(info* info_ptr){
    /*  Retrieves object names for corresponding object handle,
     *  stored in info struct
     *
     *  Calls a custom function in VRep main script, which takes in an
     *  object handle and returns the object's name.
    */
    int replySize[1] = {1};
    info_ptr->objectNames = malloc(sizeof(char*)*info_ptr->objectCount);

    for(int i = 0; i < info_ptr->objectCount; i++){
        int ret = 0;
        simxChar* replyData;
        simxInt num = i;
        info_ptr->isJoint[i] = 0;
        ret = simxCallScriptFunction(info_ptr->clientID, "", sim_scripttype_mainscript,
                "get_object_names", 1, &num, 0, NULL, 0, NULL, 0, NULL, 
                NULL, NULL, NULL, NULL, &replySize, &replyData, NULL, NULL,
                simx_opmode_blocking);
        if (ret != simx_return_ok){
            printf("ret not ok\n");
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
            token = strtok(NULL, underScore);
            if (count == 5){
                info_ptr->isJoint[i] = 1;
            }
        }
    }
    printf("Got object names\n");
}

void get_joint_angles_vrep(info* info_ptr, move* move_ptr) {
	/* Updates info struct with all jaco arm joint current angles 
	*  joints 4 and 5 are initially adjusted to take on FK values,
	*  the other joints are offset by 180 degrees */
	
	int count = 0;
	while (count < 6){
			printf(".%d\n", info_ptr->jacoArmJointHandles[count]);
			simxFloat position;
			int ret = simxGetJointPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[count], &position, simx_opmode_blocking);
			move_ptr->currAng[count] = 0;
			if ((count == (5 - 1)) || (count == (4-1))) {
				move_ptr->currAng[count] = (double)position;
			}
			else {
				move_ptr->currAng[count] = (double)position - 3.141592;
				if (move_ptr->currAng[count] < 0) {
					move_ptr->currAng[count] += 2 * 3.141592;
				}
			}
			
			printf("this is angle: %d, %f, %f\n", info_ptr->jacoArmJointHandles[count], position, move_ptr->currAng[count]);
			//printf("this is angle: %s, %f\n", info_ptr->objectNames[info_ptr->jacoArmJointHandles[count]], position);
			++count;
	}
}

void initial_arm_config_vrep(info* info_ptr, move* move_ptr) {
	/* puts the arm into a starting pos, straight up, by rotating joints 4 and 5
	*  the joint handles are also found, and angles of each using get_joint_angles */

	int i = 0;
	int num = 0;
	info_ptr->jacoArmJointHandles = malloc(sizeof(int) * 6);
	while (num < 6) {

		if (info_ptr->isJoint[i]) {
			//printf("is joint: %s\n", info_ptr->objectNames[i]);
			info_ptr->jacoArmJointHandles[num] = info_ptr->objectHandles[i];
			printf("joint handle: %d\n", info_ptr->jacoArmJointHandles[num]);
			++num;
		}
		++i;
	}

	get_joint_angles_vrep(info_ptr, move_ptr);
	
	int ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[5-1], 5, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[5-1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[5 - 1], 0, simx_opmode_oneshot_wait);
	
	ret = simxSetJointTargetVelocity(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 5, simx_opmode_oneshot_wait);
	ret = simxSetJointForce(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 25, simx_opmode_oneshot_wait);
	ret = simxSetJointTargetPosition(info_ptr->clientID, info_ptr->jacoArmJointHandles[4 - 1], 0, simx_opmode_oneshot_wait);

}

void write_object_info(info* info_ptr, char* filename){

    /*  Writes all the object handles and corresponding names to filename
     *  if filename already exists, it clears the file before writing
     *  filename is either an input argument or by default called "objects"
     */

    FILE* object_fp = fopen(filename, "w+");
    if (object_fp == NULL){
        fprintf(stdout, "Failed to generate file\n");
        exit(1);
    }
    
    for (int i = 0; i < info_ptr->objectCount; i++){
        if (info_ptr->isJoint[i]){ 
            char line[256];
            sprintf(line, "%d %s\n", i, info_ptr->objectNames[i]);
            fputs(line, object_fp);
        }

    }
    fflush(object_fp);
    fclose(object_fp);

}


void define_classic_parameters(move* move_ptr, info* info_ptr) {

	/* Define joint lengths */
	move_ptr->lengthD = malloc(sizeof(double) * 7);
	move_ptr->lengthD[0] = 9.8;			// hand offset
	move_ptr->lengthD[1] = 275.5;		// length of joint 1
	move_ptr->lengthD[2] = 410.0;		// length of joint 2
	move_ptr->lengthD[3] = 207.3;
	move_ptr->lengthD[4] = 74.3;
	move_ptr->lengthD[5] = 74.3;
	move_ptr->lengthD[6] = 168.7;

	/* Define DH Parameters */
	move_ptr->alpha = malloc(sizeof(double) * 7);
	move_ptr->alpha[0] = 0.0;				// redundant, will start at 1
	move_ptr->alpha[1] = 3.141592 / 2.0;
	move_ptr->alpha[2] = 3.141592;
	move_ptr->alpha[3] = 3.141592 / 2.0;
	move_ptr->alpha[4] = 55.0*3.141592/180.0;
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
	move_ptr->d_i[4] = -(move_ptr->lengthD[3] + move_ptr->lengthD[4]*aa);
	move_ptr->d_i[5] = -(move_ptr->lengthD[4]*aa + move_ptr->lengthD[5] * aa);
	move_ptr->d_i[6] = -(move_ptr->lengthD[6] + move_ptr->lengthD[5] * aa);

}



void fk_classic(move* move_ptr) {

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
	pt[0][0] = 0.0;
	pt[1][0] = 0.0;
	pt[2][0] = 1126.9;
	pt[3][0] = 0.0;

	/* Compute the transform matrix given angles*/
	double T[4][4];

	T[1 - 1][1 - 1] = 0.4698*sin(q3)*sin(q6) + 0.6710*cos(q3)*sin(q4)*sin(q6) + 0.4698*cos(q5)*sin(q3)*sin(q6) + 0.81915*cos(q6)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*cos(q5)*cos(q6);
	T[1 - 1][2 - 1] = 0.81915*sin(q3)*sin(q5)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q6) + 0.5736*cos(q3)*cos(q4)*cos(q6)*sin(q5) + 0.32899*cos(q3)*cos(q5)*cos(q6)*sin(q4);
	T[1 - 1][3 - 1] = 0.67101*cos(q5)*sin(q3);
	T[1 - 1][4 - 1] = 141.3028*cos(q5)*sin(q3);
	T[2 - 1][1 - 1] = 0.67101*sin(q3)*sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6)*sin(q3);
	T[2 - 1][2 - 1] = 0.46985*cos(q3)*cos(q6) + 0.46985*cos(q3)*cos(q5)*cos(q6) + cos(q4)*cos(q5)*sin(q3)*sin(q6) + 0.57358*cos(q4)*cos(q6)*sin(q3)*sin(q5) + 0.32899*cos(q5)*cos(q6)*sin(q3)*sin(q4);
	T[2 - 1][3 - 1] = 0.32899*cos(q3);
	T[2 - 1][4 - 1] = 366.50701*cos(q3);
	T[3 - 1][1 - 1] = 0.32899*cos(q4)*cos(q5)*sin(q6) + 0.57358*cos(q4)*cos(q6)*sin(q5) + cos(q5)*cos(q6)*sin(q4);
	T[3 - 1][2 - 1] = 0.67101*cos(q4)*cos(q6) + 0.57358*cos(q4)*sin(q5)*sin(q6) + cos(q5)*sin(q4)*sin(q6) + 0.57358*cos(q6)*sin(q4)*sin(q5);
	T[3 - 1][3 - 1] = 0.46985*cos(q4) + 0.46985*cos(q4)*cos(q5);
	T[3 - 1][4 - 1] = 167.55713*cos(q4) + 98.94129*cos(q4)*cos(q5);
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

	cos_tha = (px*(a + b*cos(th_b)) + py*b*sin(th_b)) / delta;
	sin_tha = (py*(a + b*cos(th_b)) - px*b*sin(th_b)) / delta;

	th_a = atan2(sin_tha, cos_tha);

	k = ((px*px) + (py*py) - (a*a) - (b*b)) / (2.0 * a*b);
	th_b = -1.0*acos(pow((k*k),0.5));

	th_c = acos((px - a*cos(th_a) - b*cos(th_a + th_b)) / c) - th_a - th_b;

}

int forward_xy_a(move* move_ptr) {

	double c;

	double ang = (3.141592 / 2.0) - move_ptr->currAng[2];
	double H = c / cos(move_ptr->currAng[3]);


	return 0;

}






