/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-22 17:59:41
*/

#include "pid.h"

void pid_set_param(PID_t* pid, float kp, float ki, float kd, bool direction)
{
        float sample_time_inSec = pid->sample_time / 1000.0;

        pid->kp = kp;
        pid->ki = ki * sample_time_inSec;
        pid->kd = kd / sample_time_inSec;

        if (direction == PID_REVERSE) {
                pid->kp = -pid->kp;
                pid->ki = -pid->ki;
                pid->kd = -pid->kd;
        }
}

float pid_compute(PID_t* pid, float set_point, float input)
{
        float error, d_input, output;

        error = set_point - input;

        pid->last_process_value += (pid->ki * error);

        if (pid->last_process_value > pid->out_max)
                pid->last_process_value = pid->out_max;

        if (pid->last_process_value < pid->out_min)
                pid->last_process_value = pid->out_min;

        d_input = input - pid->last_input;
        output = pid->kp * error + pid->last_process_value - pid->kd * d_input;

        if (output > pid->out_max)
                output = pid->out_max;
        if (output < pid->out_min)
                output = pid->out_min;

        pid->last_input = input;

        return output;
}
