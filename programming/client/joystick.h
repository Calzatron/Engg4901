#ifndef JOYSTICK_H_
#define JOYSTICK_H_
/*	Define motion struct	*/


void interpret_joystick(char* buffer);

void CreateChildProcess(void);

int joystick_input_available(void);

void joystick_clear_input_buffer(void);

int* joystick_get_char(void);

void add_to_buffer(void);

uint8_t joystickEnabled(void);

#endif // !JOYSTICK.H
