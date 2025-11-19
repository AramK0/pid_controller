


typedef struct{


    float kp, ki, kd; // gains


    float tau; // low-pass filter time constant


    float limit_min, limit_max; // the output limits

    float limit_min_int, limit_max_int; // integrator limits

    float time; // time in seconds

    float integrator, prev_error; // prev error needed for the integrator
    float differentiator, prev_measurement; // prev measure is needed for the differentiator

    float output;


} pid_controller;

void pid_init(pid_controller *pid);
float pid_update(pid_controller *pid, float setpoint, float measurement);
