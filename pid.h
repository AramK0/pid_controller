#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


typedef struct {

    /*controller gains*/

    float kp, ki, kd;


    /* Derivative low-pass filter time constant*/
    float tau;

    








}