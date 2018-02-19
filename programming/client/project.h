
#ifndef PROJECT.H

/*	Define motion struct	*/


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





#endif // !PROJECT.H
