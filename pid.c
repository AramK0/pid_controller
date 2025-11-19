#include "pid.h"





void pid_init(pid_controller *pid){
    pid->integrator = 0;
    pid->prev_error = 0;
    pid->differentiator = 0;
    pid->prev_measurement = 0;

    pid->output = 0;
}


float pid_update(pid_controller * pid, float setpoint, float measurement){

    float error = setpoint - measurement;

    float proportional = pid->kp * error;


    // integrator formula: delat Ik = Ki * T * (ek + ek-1) /2
    pid->integrator += pid->ki * 0.5f * pid->time * (error + pid->prev_error);


    if(pid->integrator > pid->limit_max_int){
        pid->integrator = pid->limit_max_int;
    }
    else if(pid->integrator < pid->limit_min_int){
        pid->integrator = pid->limit_min_int;
    }

    pid->differentiator  = -(2.0f * pid->kd * (measurement - pid->prev_measurement))
                        +   (2.0f * pid->tau - pid->time) * pid->differentiator
                        / (2.0f * pid->tau + pid->time);


    pid->output = proportional + pid->integrator + pid->differentiator;

    if(pid->output > pid->limit_max){
        pid->output = pid->limit_max;
    }
    else if(pid->output < pid->limit_min){
        pid->output = pid->limit_min;
    }

    pid->prev_error = error;
    pid->prev_measurement = measurement;
    
    return pid->output;

}



