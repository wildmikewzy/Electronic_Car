/*
 * speed_control.c
 *
 *  Created on: 2026年5月3日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
// ================变量定义=======================
PID_t left_speed,right_speed;        //左右电机速度环结构体
extern float left_motor_speed;
extern float right_motor_speed;
//========================函数定义===========================
/**
 * @brief 电机初始化
 */
void motor_init(void){
    small_driver_uart_init();       // 初始化驱动通讯功能
}
/**
 * @brief 速度闭环初始化
 */
void speed_control_init(void){

    //设定left_speed和right_speed的pid参数
    left_speed.kp = 40;
    left_speed.ki = 65;
    left_speed.kd = 120;

    right_speed.kp = 40;
    right_speed.ki = 65;
    right_speed.kd = 120;
    //初始化结构体
    left_speed.err = 0;
    left_speed.last_err = 0;
    left_speed.prev_err = 0;
    left_speed.output = 0;
    left_speed.target_val = 0;

    right_speed.err = 0;
    right_speed.last_err = 0;
    right_speed.prev_err = 0;
    right_speed.output = 0;
    right_speed.target_val = 0;
}
/**
 * @brief 速度环PID计算函数（增量式pid）
 * @param pid速度环结构体
 * @param measure_val 测量到的真是速度数值
 */
int16 PID_Compute(PID_t *pid, float measure_val) {
    pid->err = pid->target_val - measure_val;

    // 计算增量
    float increment = pid->kp * (pid->err - pid->last_err) +
                      pid->ki * pid->err +
                      pid->kd * (pid->err - 2 * pid->last_err + pid->prev_err);

    pid->output += increment;

    // 限幅，防止占空比数值超过 3000
    if(pid->output > 3000) pid->output = 3000;
    if(pid->output < -3000)  pid->output = -3000;

    pid->prev_err = pid->last_err;      //更新
    pid->last_err = pid->err;

    return pid->output;
}
/**
 * @brief 左轮速度闭环
 * @param target_speed 目标速度
 */
float speed_control_left_duty(float target_speed){

    left_speed.target_val = target_speed;
    left_speed.output = PID_Compute(&left_speed,left_motor_speed);        //左轮速度闭环控
    return -left_speed.output;
}
/**
 * @brief 右轮速度闭环
 * @param target_speed 目标速度
 */
float speed_control_right_duty(float target_speed){
    right_speed.target_val = target_speed;
    right_speed.output = PID_Compute(&right_speed,right_motor_speed);        //右轮速度闭环控制
    return right_speed.output;
}
