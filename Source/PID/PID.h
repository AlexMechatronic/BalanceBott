/* 
https://github.com/kellpossible/RobotArm
Created by Luke Frisken.

*/

#ifndef PID_H
#define PID_H

#define MAX_PREV_ERRORS 10

struct PID {
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float derivative;
	float previous_errors[MAX_PREV_ERRORS];
	float setpoint;
	char is_SS; //is steady state
	float SS_threshold; //steady state threshold
};

typedef struct PID PID;

void PID_init(PID* this, float Kp, float Ki, float Kd, float SS_threshold);
			
float PID_update(PID* this, float measured_value, float dt);

void PID_set_setpoint(PID* this, float setpoint);
float PID_get_Kp(PID* this);

void PID_set_Kp(PID* this, float Kp);
void PID_set_Ki(PID* this, float Ki);
void PID_set_Kd(PID* this, float Kd);

//returns a boolean depending on whether the PID is in a steady state
char PID_is_steady_state(PID* this);

char PID_calc_steady_state(PID* this);
void PID_push_prev_error(PID* this, float error);
float PID_get_avg_error(PID* this);
float PID_get_prev_error(PID* this);
void PID_clear_prev_errors(PID* this);
void PID_set_SS_threshold(PID* this, float SS_threshold);

void PID_print_errors(PID* this);

#endif //PID_H