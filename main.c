#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"


int main(){

    pid_controller pid;

    pid.kp = 3.0f;
    pid.ki = 0.8f;
    pid.kd = 1.2f;
    pid.tau = 0.02f;
    pid.limitMin = -10.0f;
    pid.limitMax = 10.0f;
    pid.limitMinInt = -5.0f;
    pid.limitMaxInt = 5.0f;
    pid.T = 0.01f; // 10 ms loop

    pid_controller_init(&pid);
    
    float setpoint = 0.0f; /* Target pitch (degree) of the drone we want it to be sat 0 deg for flat*/
    float pitch = 10.0f; // starts tilted 10 degress forward
    float pitch_rate = 0.0f; // angular velocity
    float torque;
    float inertia = 1.0f; // describes how hard it is for the drone to tilt
    float damping = 0.3f;


    FILE *fl = fopen("drone_pid.csv", "w");
    fprintf(fl, "time,setpoint,output\n");

    for(int i = 0; i < 100; i++){
        torque = pid_controller_update(&pid, setpoint, pitch);


        // angular acceleration = (torque / inertia) - damping * rate
        float accel = (torque / inertia) - damping * pitch_rate;

        pitch_rate = pitch_rate + accel * pid.T;
        pitch = pitch + pitch_rate * pid.T;

        fprintf(fl, "%.3f,%.3f, %.3f, %.3f\n", i*pid.T, setpoint, pitch, torque);

    }

    fclose(fl);
    printf("drone pitch simulation copied to drone_pid.csv\n");



    return 0;
}