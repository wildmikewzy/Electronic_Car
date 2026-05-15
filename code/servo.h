/*
 * servo.h
 *
 *  Created on: 2026年5月13日
 *      Author: 28752
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_



                                                  // 舵机动作角度
//float servo_motor_dir = 1;                                                      // 舵机动作状态

// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ----------s--------
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(1+(float)(x)/180.0))
#define SERVO_MOTOR_PWM_1             (ATOM1_CH2_P02_2)                           // 大臂对应的舵机所占用的PWM通道
#define SERVO_MOTOR_PWM_2             (ATOM1_CH4_P02_4)                           // 小臂对应的舵机所占用的PWM通道
#define SERVO_MOTOR_FREQ            (50)                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (65)                                       // 定义主板上舵机活动范围 角度     最低的角度  60
#define SERVO_MOTOR_R_MAX           (220)                                       // 定义主板上舵机活动范围 角度     最高的角度  220

#define ARRIVE_THRESHOLD (0.5f)   // 判断到位的容差

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif
//=================函数声明=======================
void servo_init(void);
void servo_smooth_move(float);
void servo_control_repeat(void);
void servo_position_up(void);
void servo_position_down(void);
void servo_set_target_up(void);
void servo_set_target_down(void);
#endif /* CODE_SERVO_H_ */
