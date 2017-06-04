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

void PID_init(PID* ths, float Kp, float Ki, float Kd, float SS_threshold);
			
float PID_update(PID* ths, float measured_value, float dt);

void PID_set_setpoint(PID* ths, float setpoint);
float PID_get_Kp(PID* ths);

void PID_set_Kp(PID* ths, float Kp);
void PID_set_Ki(PID* ths, float Ki);
void PID_set_Kd(PID* ths, float Kd);

//returns a boolean depending on whether the PID is in a steady state
char PID_is_steady_state(PID* ths);

char PID_calc_steady_state(PID* ths);
void PID_push_prev_error(PID* ths, float error);
float PID_get_avg_error(PID* ths);
float PID_get_prev_error(PID* ths);
void PID_clear_prev_errors(PID* ths);
void PID_set_SS_threshold(PID* ths, float SS_threshold);

void PID_print_errors(PID* ths);

#endif //PID_H