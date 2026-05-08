/*
 * common.h
 *
 *  Created on: 2026年5月4日
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_COMMON_H_
#define CODE_COMMON_H_

//====================结构体声明=====================
typedef struct {
    float kp, ki, kd;
    float target_val;       // 目标速度
    float err;              // 当前误差
    float last_err;         // 上次误差
    float prev_err;         // 上上次误差
    int16 output;      // 输出占空比 int类型
    float output_f;     //输出占空比 float类型
} PID_t;
//=================宏定义=======================
#define PIT_t (5)               //终端周期 5ms
#define k_speed (0.003435)      //编码器-速度转换系数
#define RAD_TO_DEG          (57.2957795131f)
#define DEG_TO_RAD          (0.01745329251f)
//拨码开关
#define SWITCH1                 (P33_9)
#define SWITCH2                 (P33_11)
#define SWITCH3                 (P33_12)
//蜂鸣器
#define BUZZER_PIN              (P33_10)
//按键上下左右版本
#define KEY_DOWN                (P20_7)
#define KEY_UP                  (P20_6)
#define KEY_LEFT                (P11_6)
#define KEY_RIGHT               (P11_2)
#define KEY_ENTER               (P11_3)

#endif /* CODE_COMMON_H_ */
