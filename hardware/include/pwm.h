/*
* @Author: Trung Kien
* @Date:   2016-12-15 20:58:04
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-27 10:20:25
*/

/*
 * Implement PWM by used Timer1 in pwm mode.
 * This module can generate PWM signals with 4 different duty cycles.
 * The output as below:
 *     - The Channel 1 output is GPIOA_8
 *     - The Channel 2 output is GPIOA_9
 *     - The Channel 3 output is GPIOA_10
 *     - The Channel 4 output is GPIOA_11
 * Duty cycle canbe set in range [0..100]
 */

#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f0xx.h"

#define PWM_CHANNEL_1 (0x01)    /* Output on PA8*/
#define PWM_CHANNEL_2 (0x02)    /* Output on PA9*/
#define PWM_CHANNEL_3 (0x04)    /* Output on PA10*/
#define PWM_CHANNEL_4 (0x08)    /* Output on PA11*/

/* Init pwm module, channels canbe init as multi channel.
*  Eg:
*       pwm_init(PWM_CHANNEL1 + PWM_CHANNEL_2, 10000);
*/
void pwm_init(uint8_t channels, uint16_t frequency);

/* Set duty cycle in [0 : 100]*/
void pwm_set_duty(uint8_t channels, uint16_t duty_cycle);

#endif //__PWM_H__
