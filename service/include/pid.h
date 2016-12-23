/*
* @Author: Kienltb
* @Date:   2016-12-22 14:27:49
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-22 14:42:24
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
        float last_process_value;
        float sample_time; /*ms*/
        bool  direction; /* PID_DIRECT or PID_REVERSE*/
}PID_t;

void pid_set_param(PID_t* pid, float kp, float ki, float kd, bool direction);

float pid_compute(PID_t* pid, float setpoint, float input);

#endif //__PID_H__

