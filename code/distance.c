/*
 * dist_pidtance.c
 *
 *  Created on: 2026年5月7日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
#include "common.h"
#include "distance.h"
#include "math.h"


PID_t dist_pid;      //距离环PID结构体
Pose_t car_pose = {0, 0, 0};        //小车位置结构体
/**
 * @brief 距离环PID结构体初始化
 */
void distance_PID_init(void){
    dist_pid.err = 0.0;
    dist_pid.kp = 0.8;
    dist_pid.ki = 0.0;
    dist_pid.kd = 1.0;
    dist_pid.last_err = 0.0;
    dist_pid.output = 0;
    dist_pid.output_f = 0.0;
    dist_pid.prev_err = 0.0;
    dist_pid.target_val = 0.0;
}
/**
 * @brief 距离环PID
 */
float distance_control(float target_dist, float current_dist) {
    dist_pid.err = target_dist - current_dist;

    // 距离环计算 (位置式P控制)
    dist_pid.output_f = dist_pid.kp * dist_pid.err+dist_pid.kd * (dist_pid.err - dist_pid.last_err);
    dist_pid.last_err = dist_pid.err;
    // 速度限幅：防止离目标太远时起步太猛
    if (dist_pid.output_f > MAX_SPEED_LIMIT) dist_pid.output_f = MAX_SPEED_LIMIT;
    if (dist_pid.output_f < -MAX_SPEED_LIMIT) dist_pid.output_f = -MAX_SPEED_LIMIT;

    // 停止死区：距离目标不到 1cm 就彻底停下，防止反复挪动
    if (fabsf(dist_pid.err) < 0.01f) {
        dist_pid.output_f = 0;
    }
    return dist_pid.output_f;
}


void update_position(float current_yaw, float total_dist) {
    // 1. 计算当前时刻的位移增量 dS
    float dS = total_dist - car_pose.last_dist;
    car_pose.last_dist = total_dist;

    // 2. 角度转弧度 (角度 * PI / 180)
    // 假设 yaw=0 时指向 X 轴正方向，逆时针为正
    float rad = current_yaw * 3.14159265f / 180.0f;

    // 3. 累加坐标
    car_pose.x += dS * cosf(rad);
    car_pose.y += dS * sinf(rad);
}

