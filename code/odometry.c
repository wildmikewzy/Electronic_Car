/*
 * odometry.c
 *
 *  Created on: 2026年5月5日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
// 定义全局变量
float total_encoder_ticks = 0.0;    //编码器脉冲数积分值
float current_delta_ticks = 0.0;   // 当前周期的编码器脉冲增量
float DT = 0.005;       //5ms中断周期对应的实践微分
float left_motor_speed = 0.0,right_motor_speed = 0.0,speed = 0.0;
float distance;     //对速度积分得到路程
/**
 * @brief 更新编码器脉冲更新率，并对其进行积分，用来对应相应的路程
 */
void update_odometry_encoder(void)
{
    // 1. 获取当前周期的编码器增量(编码器速度）（由硬件接口读取）
    // 假设这个函数返回自上次读取以来的脉冲数
    current_delta_ticks = motor_value.receive_left_speed_data;
    // 2. 积分：直接累加增量
    // 绝对值累加适用于测量路程（不分前进后退）
    // 如果是测量坐标位移，则直接加（不带abs）
    total_encoder_ticks += current_delta_ticks*DT;
}

/**
 * @brief 在标定开始前调用的重置函数
 */
void reset_odometry(void)
{
    total_encoder_ticks = 0.0;
}
/**
 * @brief 速度计算函数
 */
float calc_speed(void){
    left_motor_speed = motor_value.receive_left_speed_data * k_speed;
    right_motor_speed = (-motor_value.receive_right_speed_data) * k_speed;
    speed = (left_motor_speed + right_motor_speed) / 2.0;
    return speed;
}
/**
 * @brief 路程计算函数（对速度进行积分）
 */
float calc_distance(void){
    distance += speed * DT;
    return distance;
}


