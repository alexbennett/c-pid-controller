/*
 * pid.c
 *
 * Created: 11/15/2017 2:26:36 AM
 *  Author: Alex Bennett
 */ 

#include <asf.h>
#include "pid.h"

// System tick tracking
extern volatile uint32_t g_ul_us_ticks;

pid_return_code_t pid_init(pid_controller_t* pid, float kp, float ki, float kd, float output_min, float output_max, float* input, float setpoint, pid_controller_mode_t pid_mode)
{
    // Error checking
    if(kp < 0 || ki < 0 || kd < 0) return PID_BAD_TUNING_ERROR;
    
    // Set mode
    pid->pid_mode = pid_mode;
    
    // Set update frequency
    pid->update_freq = 10;
    
    // Set tunings
    pid->kp = kp;
    pid->ki = ki * (1 / (float) pid->update_freq);
    pid->kd = kd / (1 / (float) pid->update_freq);
    
    // Setup references
    pid->input = input;
    
    // Setup constraints
    pid->setpoint = setpoint;
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    // Initialize remaining variables
    pid->last_input = 0.0f;
    pid->output_sum = 0.0f;
    
    // Set initial last update
    pid->last_update = (g_ul_us_ticks / 1000) - (1 / pid->update_freq * 1000);
    
    return PID_SUCCESS;
}

pid_return_code_t pid_compute(pid_controller_t* pid)
{
    // Current time
    uint32_t curr_time = g_ul_us_ticks;
    
    // Ensure correct update frequency
    uint32_t delta_t = (curr_time / 1000) - pid->last_update;
    
    // Only continue if elapsed time is greater than sample rate
    if(delta_t >= (1 / pid->update_freq * 1000))
    {        
        // Grab variables from pointers
        float input = *pid->input;
      
        // Compute error
        float err = pid->setpoint - input;
        
        // Calculate input derivative
        float d_input = input - pid->last_input;
        
        // Update integral
        pid->output_sum += pid->ki * err;
                
        // Proportional on measurement (P_ON_M)
        if(pid->pid_mode == PID_MODE_PONM)
        {
            pid->output_sum -= pid->kp * d_input;
        }
        
        // Constrain output summation
        if(pid->output_sum > pid->output_max)
        {
            pid->output_sum = pid->output_max;
        }
        else if(pid->output_sum < pid->output_min)
        {
            pid->output_sum = pid->output_min;
        }
        
        // Create output
        float output = 0.0f;
        
        // Apply proportional on error (P_ON_E) if applicable
        if(pid->pid_mode == PID_MODE_PONE)
        {
            output = pid->kp * err;
        }
        else
        {
            output = 0.0f;
        }
        
        // Finish output calculation
        output += pid->output_sum - pid->kd * d_input;
        
        // Constrain output
        if(output > pid->output_max)
        {
            output = pid->output_max;
        }
        else if(pid->output_sum < pid->output_min)
        {
            output = pid->output_min;
        }
                
        // Set output to referenced output
        pid->output = output;
        
        // Update tracking variables
        pid->last_input = input;
        pid->last_update = curr_time / 1000;
        
        // Success
        return PID_SUCCESS;
    }
    else
    {
        // Not time to update
        return PID_NOT_COMPUTED;
    }
}