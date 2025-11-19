#include <stdio.h>
#include <math.h>

#include "pid.h"

int main(){


    pid_controller pid;

    pid.kp = 5.0f;
    pid.ki = 1.5f;
    pid.kd = 2.0f;
    pid.tau = 0.02f;
    pid.limit_min = -10.0f;
    pid.limit_max = 10.0f;
    pid.limit_min_int = -5.0f;
    pid.limit_max_int = 5.0f;
    pid.time = 0.01f; // 0.01 seconds per loop;



    pid_init(&pid);

    float setpoint = 5.0f; // Target pitcj (dgree ) of the drone
    float pitch = 0.0f;
    float pitch_rate = 0.0f;
    float torque;
    float inertia = 1.0f;
    float damping = 0.3f;

    FILE *file = fopen("drone_pid.csv", "w");
    fprintf(file, "Time : Setpoint : Output\n");

    for(int i = 0; i < 20000; i++){
        torque = pid_update(&pid, setpoint, pitch);

        float acceleration = (torque / inertia) - damping * pitch_rate;


        pitch_rate = pitch_rate + acceleration * pid.time;
        pitch = pitch + pitch_rate * pid.time;

        fprintf(file, "%.3f : %.3f : %.3f\n", i*pid.time, setpoint, pitch);

    }
    fclose(file);




    return 0;
}
