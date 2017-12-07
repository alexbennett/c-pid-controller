/*
 * pid.h
 *
 * Created: 11/15/2017 10:57:40 PM
 *  Author: Alex Bennett
 */ 

#ifndef PID_H_
#define PID_H_

enum pid_return_code {
    PID_SUCCESS = 0,
    PID_NOT_COMPUTED = 1,
    PID_BAD_TUNING_ERROR = 2
} typedef pid_return_code_t;

enum pid_controller_mode
{
    PID_MODE_PONM = 0,
    PID_MODE_PONE = 1
} typedef pid_controller_mode_t;

struct pid_controller {
    // Tuning parameters
    float kp;
    float ki;
    float kd;
    
    // Limits
    float output_min;
    float output_max;
    
    // I/O
    float* input;
    float  last_input;
    float output;
    float output_sum;
    float setpoint;

    // Timing
    uint32_t last_update;
    uint16_t update_freq;
    
    // Configuration
    pid_controller_mode_t pid_mode;
} typedef pid_controller_t;

pid_return_code_t pid_init(pid_controller_t* pid, float kp, float ki, float kd, float output_min, float output_max, float* input, float setpoint, pid_controller_mode_t pid_mode);
pid_return_code_t pid_set_tunings(pid_controller_t* pid, float kp, float ki, float kd);
pid_return_code_t pid_compute(pid_controller_t* pid);

#endif /* PID_H_ */