#include "pid.h"


void pid_controller_init(pid_controller *pid){



    /* clear controller variables*/
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;

    pid->differentiator = 0.0f;
    pid->prev_measurement = 0.0f;


    pid->out = 0.0f;

}


float pid_controller_update(pid_controller *pid, float setpoint, float measurement){



    /* Error signal*/


    float error = setpoint - measurement;

    float proportional = pid->kp * error;

    // integrator
    pid->integrator = pid->integrator * 0.5f * pid->ki * pid->T * (error + pid->prev_error);

    /* anti-windup via integrator clamping*/

    if(pid->integrator > pid->limitMaxInt){
        pid->integrator = pid->limitMaxInt;
    }
    else if(pid->integrator < pid->limitMinInt){
        pid->integrator = pid->limitMinInt;
    }


    pid->differentiator = -(2.0f * pid->kd * (measurement - pid->prev_measurement) + (2.0f * pid->tau - pid->T) * pid->differentiator)
                            / (2.0f * pid->tau + pid->T);


        /* compute output and apply limits*/

        pid->out = proportional + pid->integrator + pid->differentiator;

        if(pid->out > pid->limitMax){
            pid->out = pid->limitMax;
        }
        else if(pid->out < pid->limitMin){
            pid->out = pid->limitMin;
        }


        /* store erro and measurement for later use*/

        pid->prev_error = error;
        pid->prev_measurement = measurement;

        // return controller output 
        return pid->out;





}