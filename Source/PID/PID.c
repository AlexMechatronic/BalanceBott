#include "PID.h"
#include "Types.h"
#include "Utilities.h"

#include <TermIO.h>
#include <stdio.h>

#define DEFAULT_ERROR 1.0f

//push an error onto the previous error stack
//and rotate all the previous errors
void PID_push_prev_error(PID* this, float error){
  int i;
  float previous_error;
  for(i=0;i<(MAX_PREV_ERRORS - 1);i++){
    this->previous_errors[i+1] = this->previous_errors[i];
  }
  this->previous_errors[0]=error;
}

float PID_get_avg_error(PID* this){
 int i;
 float sum=0.0f;
 for(i=0;i<MAX_PREV_ERRORS;i++){
  sum += this->previous_errors[i];
 }
 return sum/((float)MAX_PREV_ERRORS);
}

float PID_get_prev_error(PID* this){
 return this->previous_errors[0]; 
}

void PID_print_errors(PID* this){
  int i;
  printf("Errors:");
  for(i=0;i<MAX_PREV_ERRORS;i++){
    printf(" %f",this->previous_errors[i]);
  }
  printf("\n");
}

 //set all previous errors on the stack to DEFAULT_ERROR
void PID_clear_prev_errors(PID* this){
	int i;
	for(i=0;i<MAX_PREV_ERRORS;i++){
	  this->previous_errors[i] = DEFAULT_ERROR;
	} 
}

void PID_init(PID* this, float Kp, float Ki, float Kd, float SS_threshold){
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->setpoint = 0;
	this->derivative = 0;
	this->integral = 0;
	
	PID_clear_prev_errors(this);
	
	this->is_SS = 0;
	this->SS_threshold = SS_threshold;
}

char PID_calc_steady_state(PID* this){
    int i;
    char above_threshold = 1;
    for(i=0;i<MAX_PREV_ERRORS;i++){
        if(abs_f(this->previous_errors[i]) > this->SS_threshold){
            above_threshold = 0;
        }
    }
    return above_threshold;
}

/*Should I be keeping track of dt in here?*/
float PID_update(PID* this, float measured_value, float dt) {
  float return_value = 0;
  
  //calculate difference between actual (measured) value
  // and desired value (setpoint)
	float error = this->setpoint - measured_value;
	
	// track error over time, scaled to the timer interval
	//this->integral = this->integral + error*dt;
	this->integral += error*dt;

	//need to multiply numerator by 1000?
  // determine the amount of change from the last time checked
	this->derivative = (error - PID_get_prev_error(this))/dt;
	PID_push_prev_error(this, error);
	
	
	//calculate is steady state...
	//printf("err: %f th: %f\n", abs_f(avg_error), this->SS_threshold);
	if(PID_calc_steady_state(this)){
		printf("is_SS\n");
		this->is_SS = 1; 
	}
	
	//calculate output value required to get to desired value
	return_value += this->Kp * error;
	return_value +=	this->Ki * this->integral;
	return_value += this->Kd * this->derivative;
	return return_value;
}
/*sensor input 0 -> 426*/

void PID_set_setpoint(PID* this, float setpoint) {
	this->setpoint = setpoint;
	this->is_SS = 0;
	PID_clear_prev_errors(this);
}

float PID_get_Kp(PID* this){
  return this->Kp;
}

void PID_set_Kp(PID* this, float Kp){
  this->Kp = Kp;
}
void PID_set_Ki(PID* this, float Ki){
  this->Ki = Ki;
}
void PID_set_Kd(PID* this, float Kd){
  this->Kd;
}

void PID_set_SS_threshold(PID* this, float SS_threshold){
  this->SS_threshold = SS_threshold;
}

//returns a boolean depending on whether the PID is in a steady state
char PID_is_steady_state(PID* this){
  return this->is_SS;
}
  