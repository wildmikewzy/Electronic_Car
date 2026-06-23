/*
 * common.h
 *
 *  Created on: 2026年5月4日
 *      Author: Wild_Mike_wzy
 */

#ifndef CODE_COMMON_H_
#define CODE_COMMON_H_
//==================枚举提定义=======================
typedef enum{       //任务状态枚举体
    TURN_OFF,    //不执行任务状态
    BASE_TASK_1,
    BASE_TASK_2,
    BASE_TASK_3,
    ADVANCE_TASK_1,
    ADVANCE_TASK_2,
    ADVANCE_TASK_3,
} taskType;
//====================结构体定义=====================
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
#define PIT_t (5)               //中断周期 5ms
#define k_speed (0.003435)      //编码器-速度转换系数
#define RAD_TO_DEG          (57.2957795131f)
#define DEG_TO_RAD          (0.01745329251f)
//拨码开关
#define SWITCH1                 (P33_9)
#define SWITCH2                 (P33_11)
#define SWITCH3                 (P33_12)
//蜂鸣器
#define BUZZER_PIN              (P33_10)
//五脚开关
#define KEY_DOWN                (P20_7)
#define KEY_UP                  (P20_6)
#define KEY_LEFT                (P11_6)
#define KEY_RIGHT               (P11_2)
#define KEY_ENTER               (P11_3)
//树莓派串口通信串口定义
#define RAS_UART                       (UART_3)
#define RAS_BAUDRATE                   (115200        )
#define RAS_RX                         (UART3_TX_P15_6)
#define RAS_TX                         (UART3_RX_P15_7)
//灰度传感器引脚定义
#define GRAY_1  (P00_1)
#define GRAY_2  (P00_0)
#define GRAY_3  (P00_3)
#define GRAY_4  (P00_2)
#define GRAY_5  (P00_5)
#define GRAY_6  (P00_4)
#define GRAY_7  (P00_7)
#define GRAY_8  (P00_6)
//电磁铁
#define MEGNET_PIN (P20_9)
//蜂鸣器
#define BUZZER_PIN              (P33_10)
#define LED1   (P21_7)
#endif /* CODE_COMMON_H_ */
