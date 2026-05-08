/*
 * direction.h
 *
 *  Created on: 2026Äê5ÔÂ6ÈÕ
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_DIRECTION_H_
#define CODE_DIRECTION_H_

#define MAX_TURN_SPEED (2)

void direction_PID_init(void);
float direction_PID(float target_yaw, float current_yaw, float gyro_z);



#endif /* CODE_DIRECTION_H_ */
