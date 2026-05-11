/*
 * task.c
 *
 *  Created on: 2026年5月10日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"

//===============外部声明====================
extern void bibi(int8 n);           //从外部声明蜂鸣器bibi函数
extern MotionState_t current_state;
extern taskType current_running_task;
//=========================================
static uint8 sub_step = 0;      //子任务分解步骤
/**
 * @brief 基础任务（1）执行代码
 */
void task1_logic(void) {
    float turn_speed = 0;
    float base_speed = 0.0;

    switch (sub_step) {
        case 0: // 状态：巡线寻 B
            if (!gray_is_lost()) {
                base_speed = 0.2f; // 巡线基础速度
                turn_speed = gray_track_PID_realize(); // 灰度 PID 修正
            } else {
                // 找不到线了，说明到了 B 点端点
                small_driver_set_duty(0, 0); // 紧急停车
                sub_step = 1;
            }
            break;

        case 1: // 状态：原地转身
            // 设定一个固定的原地旋转速度
            turn_speed = 0.05f;
            base_speed = 0;
            // 判定中间两个传感器是否碰到黑线
            // gray_get_mask() 获取 8 位掩码，0x01 对应二进制 00000001
            if (!gray_is_lost()) {
                small_driver_set_duty(0, 0);
                sub_step = 2;
            }
            break;

        case 2: // 状态：巡线回 A
            if (!gray_is_lost()) {
                base_speed = 0.2f;
                turn_speed = gray_track_PID_realize();
            } else {
                // 回到 A 点
                small_driver_set_duty(0, 0);
                sub_step = 3;
            }
            break;

        case 3: // 任务完成
            bibi(2); // 蜂鸣器响2声，声音提示
            sub_step = 0; // 重置子状态
            current_running_task = TURN_OFF;
            base_speed = 0.0f;
            turn_speed = 0.0f;
            break;
    }
    // 执行输出
    small_driver_set_duty(speed_control_left_duty(base_speed + turn_speed),
                          speed_control_right_duty(base_speed - turn_speed));
}



