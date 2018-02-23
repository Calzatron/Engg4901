#ifndef JOYSTICK_H_
#define JOYSTICK_H_
/*	Define motion struct	*/


void interpret_joystick(char* buffer);

void CreateChildProcess(void);

int joystick_input_available(void);

void joystick_clear_input_buffer(void);

int* joystick_get_char(void);


#endif // !JOYSTICK.H
