/*
 * distance.h
 *
 *  Created on: 2026年5月7日
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_DISTANCE_H_
#define CODE_DISTANCE_H_


#define MAX_SPEED_LIMIT (0.5)
// 定义坐标结构体
typedef struct {
    float x;      // 坐标 X (米)
    float y;      // 坐标 Y (米)
    float last_dist; // 记录上一时刻的总里程
} Pose_t;


//===============函数声明=================
void distance_PID_init(void);
float distance_control(float target_dist, float current_dist);
void update_position(float current_yaw, float total_dist);
#endif /* CODE_DISTANCE_H_ */
