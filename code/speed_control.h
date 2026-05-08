/*
 * speed_control.h
 *
 *  Created on: 2026年5月3日
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_SPEED_CONTROL_H_
#define CODE_SPEED_CONTROL_H_

#include "zf_common_headfile.h"
#include "common.h"
// ===================函数声明=======================
void motor_init(void);
int16 PID_Compute(PID_t *pid, float measure_val);
void speed_control_init(void);
float speed_control_left_duty(float target_speed);
float speed_control_right_duty(float target_speed);
#endif /* CODE_SPEED_CONTROL_H_ */
