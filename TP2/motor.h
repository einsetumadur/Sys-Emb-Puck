#ifndef MOTOR_H
#define MOTOR_H


void motor_init(void);
void motor_set_speed(float speed_r, float speed_l);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void motor_stop(void);

typedef struct
{
	float pos_r;
	float pos_l;
	float vel_r;
	float vel_l;
} Movement_parameters;

void motor_set_sequence(const Movement_parameters* movement_parameters, uint32_t size);

#endif /* MOTOR_H */
