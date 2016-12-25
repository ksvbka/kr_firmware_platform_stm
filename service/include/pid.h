/*
* @Author: Kienltb
* @Date:   2016-12-22 14:27:49
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-25 17:11:11
*/

/**
 * PID controller.
 * Base on  PID Library Copyright @2013 vidieukhien.net forum
 */

#ifndef __PID_H__
#define __PID_H__

#include "typedef.h"

#define PID_DIRECT      0
#define PID_REVERSE     1

typedef struct PID{
        float kp;
        float ki;
        float kd;
        float out_min;
        float out_max;
        float last_input;
        float error_accumulate;
        float sample_time; /*s*/
        bool  direction; /* PID_DIRECT or PID_REVERSE*/
}PID_t;

void pid_set_param(PID_t* pid, float kp, float ki, float kd, bool direction, float sample_time);

float pid_compute(PID_t* pid, float setpoint, float input);

#endif //__PID_H__

