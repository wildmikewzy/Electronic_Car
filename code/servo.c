#include "zf_common_headfile.h"
float servo_motor_duty = 220.0;


const float MAX_SPEED = 0.7;     // 最高速度限制
static float current_big_duty = 0.0;    // 大臂当前位置
static float current_small_duty = 0.0;  // 小臂当前位置
float big_speed = 0.0;           // 大臂速度
const float BIG_ACC = 0.03;     // 大臂运动加速度
const float SMALL_FOLLOW_K = 0.15; // 关键：小臂跟随系数（0.1-0.2之间）
/**
 * @brief 舵机运动控制(需要放在循环或者中断当中）
 */
void servo_smooth_move(float target_duty) {
    // --- 1. 大臂使用 S 曲线加减速 ---
    float error_big = target_duty - current_big_duty;
    if (abs(error_big) > 0.1) {
        if (error_big > 0) {
            big_speed += BIG_ACC;
            if (big_speed > MAX_SPEED) big_speed = MAX_SPEED;
        } else {
            big_speed -= BIG_ACC;
            if (big_speed < -MAX_SPEED) big_speed = -MAX_SPEED;
        }
        // 减速逻辑保留
        if (abs(error_big) < abs(big_speed * 10)) big_speed *= 0.8;
        current_big_duty += big_speed;
    } else {
        big_speed = 0;
        current_big_duty = target_duty;
    }
    // --- 2. 小臂引入“弹性跟随” (一阶低通滤波) ---
    // 目标是让小臂相对于大臂保持角度不变，但它是缓慢地滑向那个位置
    float target_small = current_big_duty; // 联动目标
    // 核心公式：当前位置 = 旧位置 + (目标 - 旧位置) * 比例
    current_small_duty += (target_small - current_small_duty) * SMALL_FOLLOW_K;

    // --- 3. 最终输出与限位 ---
    // 为每个舵机设置独立的死区保护，防止负数突变
    float out1 = current_big_duty - 60.0;
    float out2 = current_small_duty;

    if(out1 < -30) out1 = -30; if(out1 > SERVO_MOTOR_R_MAX) out1 = SERVO_MOTOR_R_MAX;
    if(out2 < -30) out2 = -30; if(out2 > SERVO_MOTOR_R_MAX) out2 = SERVO_MOTOR_R_MAX;

    pwm_set_duty(SERVO_MOTOR_PWM_1, (uint32)SERVO_MOTOR_DUTY(out1));
    pwm_set_duty(SERVO_MOTOR_PWM_2, (uint32)SERVO_MOTOR_DUTY(out2));
}

/**
 * @brief 舵机往复运动函数，用于测试舵机稳定性
 * @note 在主函数中直接调用即可
 */
float target_pos = SERVO_MOTOR_R_MAX;
void servo_control_repeat(void){
    // 1. 调用平滑控制函数（执行一小步）
    servo_smooth_move(target_pos);
    // 2. 到位检测：检查当前位置是否已经接近目标
    if (abs(target_pos - current_big_duty) < ARRIVE_THRESHOLD) {

        // 到达后停留片刻，防止物体晃动（可选）
        system_delay_ms(500);
        // 3. 切换目标位置（往复逻辑）
        if (target_pos == SERVO_MOTOR_R_MAX ) {
            target_pos = SERVO_MOTOR_L_MAX ;  // 到顶了，开始向下
        } else {
            target_pos = SERVO_MOTOR_R_MAX ; // 到底了，开始向上
        }
    }
    // 4. 关键：维持舵机控制频率
    system_delay_ms(7);
}
/**
 *@brief 舵机到达最顶端位置
 */
void servo_position_up(void){
    // 确保从 0 速度开始启动，防止起步冲击
    big_speed = 0.0;
    while (abs(SERVO_MOTOR_R_MAX - current_big_duty) > ARRIVE_THRESHOLD) {
        // 调用你实测最稳的平滑函数
        servo_smooth_move(SERVO_MOTOR_R_MAX);

        // 维持你实测最稳的频率
        system_delay_ms(7);
        // 安全保险：防止死循环（比如设置一个最大计数器）
    }
    // 到达后通过停顿来抵消惯性
    system_delay_ms(300);
}
/**
 * @brief  舵机到达底端位置
 */
void servo_position_down(void){
    // 确保从 0 速度开始启动，防止起步冲击
    big_speed = 0.0;
    while (abs(SERVO_MOTOR_L_MAX - current_big_duty) > ARRIVE_THRESHOLD) {
        // 调用你实测最稳的平滑函数
        servo_smooth_move(SERVO_MOTOR_L_MAX);

        // 维持你实测最稳的频率
        system_delay_ms(7);
        // 安全保险：防止死循环（比如设置一个最大计数器）
    }
    // 到达后通过停顿来抵消惯性
    system_delay_ms(300);
}
/**
 * @brief 舵机的初始化,舵机初始姿态在低处
 */
void servo_init(void){
    pwm_init(SERVO_MOTOR_PWM_1, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR_PWM_2, SERVO_MOTOR_FREQ, 0);
    servo_position_down();

}
