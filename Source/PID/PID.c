#include "PID.h"
#include <stdio.h>

#define DEFAULT_ERROR 1.0f

//push an error onto the previous error stack
//and rotate all the previous errors
void PID_push_prev_error(PID* ths, float error){
  int i;
  float previous_error;
  for(i=0;i<(MAX_PREV_ERRORS - 1);i++){
    ths->previous_errors[i+1] = ths->previous_errors[i];
  }
  ths->previous_errors[0]=error;
}

float PID_get_avg_error(PID* ths){
 int i;
 float sum=0.0f;
 for(i=0;i<MAX_PREV_ERRORS;i++){
  sum += ths->previous_errors[i];
 }
 return sum/((float)MAX_PREV_ERRORS);
}

float PID_get_prev_error(PID* ths){
 return ths->previous_errors[0]; 
}

void PID_print_errors(PID* ths){
  int i;
  printf("Errors:");
  for(i=0;i<MAX_PREV_ERRORS;i++){
    printf(" %f",ths->previous_errors[i]);
  }
  printf("\n");
}

 //set all previous errors on the stack to DEFAULT_ERROR
void PID_clear_prev_errors(PID* ths){
	int i;
	for(i=0;i<MAX_PREV_ERRORS;i++){
	  ths->previous_errors[i] = DEFAULT_ERROR;
	} 
}

void PID_init(PID* ths, float Kp, float Ki, float Kd, float SS_threshold){
	ths->Kp = Kp;
	ths->Ki = Ki;
	ths->Kd = Kd;
	ths->setpoint = 0;
	ths->derivative = 0;
	ths->integral = 0;
	
	PID_clear_prev_errors(ths);
	
	ths->is_SS = 0;
	ths->SS_threshold = SS_threshold;
}

char PID_calc_steady_state(PID* ths){
    int i;
    char above_threshold = 1;
    for(i=0;i<MAX_PREV_ERRORS;i++){
        if(abs_f(ths->previous_errors[i]) > ths->SS_threshold){
            above_threshold = 0;
        }
    }
    return above_threshold;
}

/*Should I be keeping track of dt in here?*/
float PID_update(PID* ths, float measured_value, float dt) {
  float return_value = 0;
  
  //calculate difference between actual (measured) value
  // and desired value (setpoint)
	float error = ths->setpoint - measured_value;
	
	// track error over time, scaled to the timer interval
	//ths->integral = ths->integral + error*dt;
	ths->integral += error*dt;

	//need to multiply numerator by 1000?
  // determine the amount of change from the last time checked
	ths->derivative = (error - PID_get_prev_error(ths))/dt;
	PID_push_prev_error(ths, error);
	
	
	//calculate is steady state...
	//printf("err: %f th: %f\n", abs_f(avg_error), ths->SS_threshold);
	if(PID_calc_steady_state(ths)){
		printf("is_SS\n");
		ths->is_SS = 1; 
	}
	
	//calculate output value required to get to desired value
	return_value += ths->Kp * error;
	return_value +=	ths->Ki * ths->integral;
	return_value += ths->Kd * ths->derivative;
	return return_value;
}
/*sensor input 0 -> 426*/

void PID_set_setpoint(PID* ths, float setpoint) {
	ths->setpoint = setpoint;
	ths->is_SS = 0;
	PID_clear_prev_errors(ths);
}

float PID_get_Kp(PID* ths){
  return ths->Kp;
}

void PID_set_Kp(PID* ths, float Kp){
  ths->Kp = Kp;
}
void PID_set_Ki(PID* ths, float Ki){
  ths->Ki = Ki;
}
void PID_set_Kd(PID* ths, float Kd){
  ths->Kd;
}

void PID_set_SS_threshold(PID* ths, float SS_threshold){
  ths->SS_threshold = SS_threshold;
}

//returns a boolean depending on whether the PID is in a steady state
char PID_is_steady_state(PID* ths){
  return ths->is_SS;
}
  