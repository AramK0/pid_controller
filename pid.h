#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


typedef struct {

    /*controller gains*/

    float kp, ki, kd;


    /* Derivative low-pass filter time constant*/
    float tau;

    /* Output limits*/
    float limitMin;
    float limitMax;

    /* Integrator limits*/
    float limitMinInt;
    float limitMaxInt;
    
    /*sample time in seconds*/
    float T;

    /* Controller "memory"*/
    float integrator;
    float prev_error; // required for integrator
    float differentiator;
    float prev_measurement; // required for differentiator 

    /* controller output*/
    float out;
    


} pid_controller;


// we pass it a pointer to the structure 
void pid_controller_init(pid_controller *pid);
float pid_controller_update(pid_controller *pid , float setpoint, float measurement);

#endif