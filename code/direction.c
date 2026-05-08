/*
 * dir_pidection.c
 *
 *  Created on: 2026年5月6日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
#include "common.h"
PID_t dir_pid;      //航向环pid初始化
void direction_PID_init(void){
    dir_pid.err = 0.0;
    dir_pid.kp = 0.005;
    dir_pid.ki = 0.0;
    dir_pid.kd = 0.002;
    dir_pid.last_err = 0.0;
    dir_pid.output = 0;
    dir_pid.output_f = 0.0;
    dir_pid.prev_err = 0.0;
    dir_pid.target_val = 0.0;
}
/**
 * @brief 航向换PID，这里采用PD控制
 * @param target_yaw 目标yaw角度
 * @param current_yaw 现在的yaw角度
 * @param gyro_z 现在的z轴角速度
 * @retval 返回左右轮需要的速度差值
 */
float direction_PID(float target_yaw, float current_yaw, float gyro_z) {
    //统一输入范围：强制将 current_yaw 映射到 0-360 或保持与 target 一致
    // 假设 target_yaw 始终在 0-360 之间
    if (current_yaw < 0) current_yaw += 360.0f;
    if (current_yaw >= 360.0f) current_yaw -= 360.0f;

    dir_pid.err = target_yaw - current_yaw;

    // 1. 角度过零处理（就近转弯）
    while (dir_pid.err > 180.0f) dir_pid.err -= 360.0f;
    while (dir_pid.err < -180.0f) dir_pid.err += 360.0f;

    // 2. PD 计算
    dir_pid.output_f = (dir_pid.kp * dir_pid.err - dir_pid.kd * gyro_z);

    // 3. 静态死区补偿优化
    // 只有当偏差足以产生位移趋势时才补偿，防止抖动
    float deadzone_err = 1.0f;
    float min_start_speed = 0.15f;
    if (fabsf(dir_pid.err) > deadzone_err) {
        if (dir_pid.output_f > 0) dir_pid.output_f += (min_start_speed);
        else dir_pid.output_f -= (min_start_speed);
    } else {
        // 【核心细节】在死区内不仅要 output_f=0，还要清除速度环积分
        dir_pid.output_f = 0;
    }

    // 4. 转弯限幅
    if (dir_pid.output_f > MAX_TURN_SPEED) dir_pid.output_f = MAX_TURN_SPEED;
    if (dir_pid.output_f < -MAX_TURN_SPEED) dir_pid.output_f = -MAX_TURN_SPEED;

    return dir_pid.output_f;
}


