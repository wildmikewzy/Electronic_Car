/*
 * task.c
 *
 *  Created on: 2026年5月10日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"

//===============外部声明====================
extern void bibi(int8 n);           //从外部声明蜂鸣器bibi函数
extern taskType current_running_task;
extern float distance;
//=========================================
void stop_car(void){
    small_driver_set_duty(0,0);
    system_delay_ms(300);
}
static uint8 sub_step = 0;      //子任务分解步骤
/**
 * @brief 基础任务（1）执行代码
 */
void task1_logic(void) {
    float base_speed = 0;
    float turn_speed = 0;

    switch (sub_step) {
        case 0: // 【阶段1】前进巡线：从 A 到 B
            base_speed = 0.4f;
            turn_speed = gray_track_PID_realize();
            // 判定到达 B 点：所有传感器都看到黑线（横线）或者全部丢线（冲出了 B 点）
            if (gray_is_lost()) {
                stop_car();
                system_delay_ms(200); // 停稳
                sub_step = 1;
            }
            break;
        case 1: // 【阶段2】强制左转中继点：Yaw = 90
            base_speed = -0.04;
            turn_speed = direction_PID(90.0f, yaw, gyro_z);
            // 到达 90 度附近，切入下一阶段
            if (fabsf(get_yaw_diff(90.0f, yaw)) < 3.0f) {
                sub_step = 2;
            }
            break;

        case 2: // 【阶段3】目标航向锁定：Yaw = 190
            base_speed = -0.04;
            turn_speed = direction_PID(185.0f, yaw, gyro_z);
            // 转到 190 度，此时车头已经基本对向 A 点且偏左一点
            if (fabsf(get_yaw_diff(185.0f, yaw)) < 2.0f) {
                stop_car();
                sub_step = 3;
            }
            break;

        case 3: // 【阶段4】切回巡线回到 A
            base_speed = 0.4f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() && distance >= 1.8) {
                stop_car();
                current_running_task = TURN_OFF;
                bibi(2);
            }
            break;
    }

    // 最终电机输出
    float left_target  = base_speed - turn_speed;
    float right_target = base_speed + turn_speed;

    // 输入到你之前的速度环
    small_driver_set_duty(speed_control_left_duty(left_target),
                         speed_control_right_duty(right_target));
}
/**
 * @brief 运动控制状态重置
 */
void reset_task_variables(void) {
    sub_step = 0;              // 状态机回到第一步
    distance = 0;              // 编码器里程清零（如果你的里程是累加的）
    // 清除 PID 结构体中的中间变量（防止上次残留的 error 或 last_err 导致瞬跳）
    direction_PID_init();      // 重新初始化航向环 PID
    // 确保电机处于停止状态
    small_driver_set_duty(0, 0);
    system_delay_ms(300);
}
/**
 * @brief 基础题目（2）
 */

void task2_logic(void) {
    float base_speed = 0;
    float turn_speed = 0;
    static float step_start_dist = 0; // 记录每一段的起点里程
    float current_step_dist = distance - step_start_dist;
    switch (sub_step) {
        case 0: // 【A -> D】纯惯导走直线
            base_speed = 0.3f;
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (current_step_dist >= 1.0f) { // 1.01m
                sub_step = 1;
                step_start_dist = distance;
            }
            break;

        case 1: // 【D 点转向】转向 C 点方向
            base_speed = -0.02;
            turn_speed = direction_PID(90.0f, yaw, gyro_z); // 转向 C 点
            if (fabsf(get_yaw_diff(90.0f, yaw)) < 2.0f) {
                sub_step = 2;
                step_start_dist = distance;
            }
            break;

        case 2: // 【D -> C】沿 DC 引导线巡线
            base_speed = 0.3f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() || current_step_dist >= 1.0) { // 到达 C 点
                stop_car();
                step_start_dist = distance;
                sub_step = 3;
            }
            break;
        case 3: // 【C 点转向】转向 B 点方向
            base_speed = -0.02;
            turn_speed = direction_PID(180.0f, yaw, gyro_z); // 转向 C 点
            if (fabsf(get_yaw_diff(180.0f, yaw)) < 2.0f) {
                sub_step = 4;
                step_start_dist = distance;
            }
            break;
        case 4: // 【C -> B】纯惯导走直线
            base_speed = 0.3f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.85f) { // 累积里程判定
                sub_step = 5;
                step_start_dist = distance;
            }
            break;
        case 5: // 【B 点转向】转向 A 点方向
            base_speed = -0.03;
            turn_speed = direction_PID(270.0f, yaw, gyro_z); // 转向 C 点
            if (fabsf(get_yaw_diff(270.0f, yaw)) < 2.0f){
                sub_step = 6;
                step_start_dist = distance;
            }
            break;
        case 6: // 【B -> A】沿 BA 引导线巡线回起点
            base_speed = 0.3f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() && current_step_dist>=0.95) {
                stop_car();
                current_running_task = TURN_OFF;
                bibi(2);
            }
            break;
    }
    // 最终电机输出
    float left_target  = base_speed - turn_speed;
    float right_target = base_speed + turn_speed;

    // 输入到你之前的速度环
    small_driver_set_duty(speed_control_left_duty(left_target),
                             speed_control_right_duty(right_target));
}
/**
 * @brief 维持恒定角速度的 PID（用于绕桶）
 * @param target_gyro_z 目标角速度 (deg/s)
 */
float circle_gyro_control(float target_gyro_z, float current_gyro_z) {
    float err = target_gyro_z - current_gyro_z;
    // 这里只需要一个很小的 P 和 D 即可
    static float kp_gyro_z = 0.005f;
    return kp_gyro_z * err;
}
/**
 * @brief 基础题（3）
 */
void task3_logic(void) {
    static float step_start_dist = 0;
    //static float circle_start_yaw = 0;
    float current_step_dist = distance - step_start_dist;
    float base_speed = 0;
    float turn_speed = 0;

    switch (sub_step) {
        case 0: // [A->B]
            base_speed = 0.5f;
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (current_step_dist >= 1.0f) { // 累积里程判定
                sub_step = 1;
                step_start_dist = distance;
            }
            break;
        case 1: // 【B 点：原地右转 90 度】
            base_speed = -0.02;
            // 目标 yaw 为 B 点巡线时的 yaw 270 度 (注意过零处理)
            float target_yaw = 270.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 2;
                step_start_dist = distance;
            }
            break;
        case 2: //走一小段经过b，c区域
            base_speed = 0.4f;
            turn_speed = direction_PID(270.0f, yaw, gyro_z);
            if (current_step_dist >= 0.6) { // 累积里程判定
                sub_step = 3;
                step_start_dist = distance;
            }
            break;
        case 3:     //再转弯90度
            base_speed = -0.02;
            target_yaw = 180.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 4;
                step_start_dist = distance;
            }
            break;
        case 4: //走一小段经过c,d区域
            base_speed = 0.4f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.6) { // 累积里程判定
                sub_step = 5;
                step_start_dist = distance;
            }
            break;
        case 5:     //再转弯90度
            base_speed = -0.02;
            target_yaw = 90.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 6;
                step_start_dist = distance;
            }
            break;
        case 6: //走一小段经过d,a区域
            base_speed = 0.4f;
            turn_speed = direction_PID(90.0f, yaw, gyro_z);
            if (current_step_dist >= 0.3) { // 累积里程判定
                sub_step = 7;
                step_start_dist = distance;
            }
            break;
        case 7:     //再转弯90度
            base_speed = -0.02;
            target_yaw = 0.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 8;
                step_start_dist = distance;
            }
            break;
        case 8: //走一小段经过a,b区域
            base_speed = 0.40f;
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (current_step_dist >= 0.3) { // 累积里程判定
                sub_step = 9;
                step_start_dist = distance;
            }
            break;
        case 9:     //再转弯
            base_speed = -0.02;
            target_yaw = 290.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 10;
                step_start_dist = distance;
            }
            break;
        case 10: //走一小段到达C点
            base_speed = 0.4f;
            turn_speed = direction_PID(290.0f, yaw, gyro_z);
            if (current_step_dist >= 0.65) { // 累积里程判定
                sub_step = 11;
                step_start_dist = distance;
            }
            break;
        case 11:     //再转弯
            base_speed = -0.02;
            target_yaw = 180.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                step_start_dist = distance;
                sub_step = 12;
            }
            break;
        case 12: //走一小段避免巡线出问题
            base_speed = 0.25f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.2) { // 累积里程判定
                sub_step = 13;
                step_start_dist = distance;
            }
            break;
        case 13: // 【C -> D】沿 CD 引导线巡线
            base_speed = 0.4f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() || current_step_dist >= 0.8) { // 到达 C 点
                stop_car();
                step_start_dist = distance;
                sub_step = 14;
            }
            break;
        case 14:     //再转弯90度
            base_speed = -0.03;
            target_yaw = 85.0f;
            turn_speed = direction_PID(target_yaw, yaw, gyro_z);
            if (fabsf(get_yaw_diff(target_yaw, yaw)) < 2.0f && fabsf(gyro_z) < 5.0f) {
                stop_car();
                system_delay_ms(200); // 停稳，准备起步
                sub_step = 15;
                step_start_dist = distance;
            }
            break;
        case 15: //走一小段到达A点，完赛！
            base_speed = 0.5f;
            turn_speed = direction_PID(87.0f, yaw, gyro_z);
            if (current_step_dist >= 0.9) { // 累积里程判定
                stop_car();
                current_running_task = TURN_OFF;
                bibi(2);
            }
            break;
        }
    // 最终电机输出
    float left_target  = base_speed - turn_speed;
    float right_target = base_speed + turn_speed;
    // 输入到你之前的速度环
    small_driver_set_duty(speed_control_left_duty(left_target),
                         speed_control_right_duty(right_target));
}

