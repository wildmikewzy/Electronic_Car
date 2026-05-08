/*
 * odometry.h
 *
 *  Created on: 2026Äê5ÔÂ5ÈÕ
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_ODOMETRY_H_
#define CODE_ODOMETRY_H_


void update_odometry_encoder(void);
void reset_odometry(void);
float calc_speed(void);
float calc_distance(void);

#endif /* CODE_ODOMETRY_H_ */
