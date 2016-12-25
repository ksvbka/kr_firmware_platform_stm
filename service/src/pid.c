/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-25 17:11:45
*/

#include "pid.h"

void pid_set_param(PID_t* pid, float kp, float ki, float kd, bool direction, float sample_time)
{
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
        pid->sample_time = sample_time;

        if (direction == PID_REVERSE) {
                pid->kp = -pid->kp;
                pid->ki = -pid->ki;
                pid->kd = -pid->kd;
        }
}

float pid_compute(PID_t* pid, float set_point, float input)
{
        float error = set_point - input;
        float d_input = input - pid->last_input;
        pid->error_accumulate += error;

        float p_out = pid->kp * error;
        float i_out = pid->ki*pid->error_accumulate *pid->sample_time;
        float d_out = - (pid->kd * d_input)/pid->sample_time;

        float output = p_out + i_out + d_out;

        if (output > pid->out_max)
                output = pid->out_max;
        if (output < pid->out_min)
                output = pid->out_min;

        pid->last_input = input;

        return output;
}
