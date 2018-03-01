#ifndef KINEMATICS_H_
#define KINEMATICS_H_
/*	Define motion struct	*/
#include "project.h"

void define_classic_parameters(move* move_ptr);
void fk_classic(move* move_ptr, info* info_ptr);
void fk_mod(move* move_ptr);
void ik_RRR_arm(move* move_ptr, char* plane);

#endif // !KINEMATICS.H