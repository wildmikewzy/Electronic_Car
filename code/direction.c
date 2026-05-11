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
    dir_pid.kp = 0.06;
    dir_pid.ki = 0.0;
    dir_pid.kd = 0.040;
    dir_pid.last_err = 0.0;
    dir_pid.output = 0;
    dir_pid.output_f = 0.0;
    dir_pid.prev_err = 0.0;
    dir_pid.target_val = 0.0;
}
/**
 * @brief 计算两个 0-360 度角之间的最小偏差（带正负）
 * @param target 目标角度 (0-360)
 * @param current 当前角度 (0-360)
 * @return 返回值范围 -180 到 180 度。正值代表需要左转，负值代表需要右转（视电机极性而定）
 */
float get_yaw_diff(float target, float current) {
    float diff = target - current;

    // 过零点处理：如果差值超过 180 度，说明走反了，需要通过另一侧转弯
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}
/**
 * @brief 航向换PID，这里采用PD控制
 * @param target_yaw 目标yaw角度
 * @param current_yaw 现在的yaw角度
 * @param gyro_z 现在的z轴角速度
 * @retval 返回左右轮需要的速度差值
 */
float direction_PID(float target_yaw, float current_yaw, float gyro_z) {
    // 1. 统一输入范围
    if (current_yaw < 0) current_yaw += 360.0f;
    if (current_yaw >= 360.0f) current_yaw -= 360.0f;

    // 2. 获取带符号的角度偏差 (关键：删掉 fabsf)
    dir_pid.err = get_yaw_diff(target_yaw, current_yaw);

    // 3. 基础 PD 计算
    // 这里的 kp 决定了追线快慢，kd 决定了停止时的稳准度
    dir_pid.output_f = dir_pid.kp * dir_pid.err - dir_pid.kd * gyro_z;

    // 4. 动态前馈补偿 (解决“转弯慢”的核心)
    float ff_term = 0;
    float abs_err = fabsf(dir_pid.err);

    if (abs_err > 0.2f) {
        // 补偿值：0.12f 为基础值，误差越大给的起步力稍微多一点
        float base_ff = 0.12f;
        ff_term = (dir_pid.err > 0) ? base_ff : -base_ff;
    }
    dir_pid.output_f += ff_term;

    // 5. 转弯限幅
    if (dir_pid.output_f > MAX_TURN_SPEED) dir_pid.output_f = MAX_TURN_SPEED;
    if (dir_pid.output_f < -MAX_TURN_SPEED) dir_pid.output_f = -MAX_TURN_SPEED;

    return dir_pid.output_f;
}


