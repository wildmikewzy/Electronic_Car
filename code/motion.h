/*
 * motion.h
 *
 *  Created on: 2026年5月8日
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_MOTION_H_
#define CODE_MOTION_H_

/**
 * @brief 运动状态枚举体
 */
typedef enum {
    IDLE = 0,
    ROTATING,
    TRANSLATING,
} MotionState_t;
typedef struct{
    float x;
    float y;
}Point_t;

//======================函数声明======================
void run_motion_task(float target_y, float target_d);
void path_following_logic(void);

#endif /* CODE_MOTION_H_ */
