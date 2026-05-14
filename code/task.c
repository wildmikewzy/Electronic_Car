/*
 * task.c
 *
 *  Created on: 2026年5月10日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
#include "image.h"

//===============外部声明====================
extern taskType current_running_task;
extern float distance;
//=========================================
/**
 * @brief 停车程序
 */
void stop_car(void){
    small_driver_set_duty(0,0);
    system_delay_ms(300);
}
/**
 * @brief 声光提示程序
 */
void Buzzer_and_LED(int n){
    int8 count = 0;
    while (count<n*2)
    {
        // 此处编写需要循环执行的代码
        gpio_toggle_level(LED1);
        gpio_toggle_level(BUZZER_PIN);
        count ++;
        system_delay_ms(500);
        // 此处编写需要循环执行的代码
    }
}
static uint8 sub_step = 0;      //子任务分解步骤

// task4 速度与角速度限幅，防止视觉闭环放大导致突冲
#define TASK4_MAX_BASE_SPEED   0.20f
#define TASK4_MIN_BASE_SPEED  -0.10f
#define TASK4_MAX_TURN_SPEED   0.80f

static float task4_clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}
/**
 * @brief 基础任务（1）执行代码
 */
void task1_logic(void) {
    float base_speed = 0;
    float turn_speed = 0;
    static float step_start_dist = 0; // 记录每一段的起点里程
    float current_step_dist = distance - step_start_dist;
    switch (sub_step) {
        case 0: // 【A -> B】纯惯导走直线
            base_speed = 0.4f;
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (current_step_dist >= 1.0f) { // 1.0m
                sub_step = 1;
                step_start_dist = distance;
            }
            break;
        case 1: // 【阶段2】强制左转中继点：Yaw = 90
            base_speed = -0.01;
            turn_speed = direction_PID(90.0f, yaw, gyro_z);
            // 到达 90 度附近，切入下一阶段
            if (fabsf(get_yaw_diff(90.0f, yaw)) < 3.0f) {
                sub_step = 2;
                step_start_dist = distance;
            }
            break;
        case 2: // 【阶段3】目标航向锁定：Yaw = 180
            base_speed = -0.02;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            // 转到 180 度，此时车头已经基本对向 A 点且偏左一点
            if (fabsf(get_yaw_diff(180.0f, yaw)) < 2.0f) {
                stop_car();
                sub_step = 3;
                step_start_dist = distance;
            }
            break;
        case 3: //纯惯导走一小段
            base_speed = 0.2f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.1f) { // 累积里程判定
                sub_step = 4;
                step_start_dist = distance;
            }
            break;
        case 4: // 【阶段4】切回巡线回到 A
            base_speed = 0.3f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() && current_step_dist>= 0.9) {
                stop_car();
                current_running_task = TURN_OFF;
                Buzzer_and_LED(3);
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
    image_control_reset();     // 视觉闭环清零，避免任务切换造成突变
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
            if (current_step_dist >= 1.03f) { // 1.0m
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
        case 2: //纯惯导走一小段
            base_speed = 0.2f;
            turn_speed = direction_PID(90.0f, yaw, gyro_z);
            if (current_step_dist >= 0.1f) { // 累积里程判定
                sub_step = 3;
                step_start_dist = distance;
            }
            break;
        case 3: // 【D -> C】沿 DC 引导线巡线
            base_speed = 0.3f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() || current_step_dist >= 1.0) { // 到达 C 点
                stop_car();
                step_start_dist = distance;
                sub_step = 4;
            }
            break;
        case 4: // 【C 点转向】转向 B 点方向
            base_speed = -0.03;
            turn_speed = direction_PID(180.0f, yaw, gyro_z); // 转向 C 点
            if (fabsf(get_yaw_diff(180.0f, yaw)) < 2.0f) {
                sub_step = 5;
                step_start_dist = distance;
            }
            break;
        case 5: // 【C -> B】纯惯导走直线
            base_speed = 0.3f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.90f) { // 累积里程判定
                sub_step = 6;
                step_start_dist = distance;
            }
            break;
        case 6: // 【B 点转向】转向 A 点方向
            base_speed = -0.03;
            turn_speed = direction_PID(270.0f, yaw, gyro_z); // 转向 C 点
            if (fabsf(get_yaw_diff(270.0f, yaw)) < 2.0f){
                sub_step = 7;
                step_start_dist = distance;
            }
            break;
        case 7: //纯惯导走一小段
            base_speed = 0.2f;
            turn_speed = direction_PID(270.0f, yaw, gyro_z);
            if (current_step_dist >= 0.1f) { // 累积里程判定
                sub_step = 8;
                step_start_dist = distance;
            }
            break;
        case 8: // 【B -> A】沿 BA 引导线巡线回起点
            base_speed = 0.3f;
            turn_speed = gray_track_PID_realize();
            if (gray_is_lost() && current_step_dist>=0.80) {
                stop_car();
                current_running_task = TURN_OFF;
                Buzzer_and_LED(3);
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
            base_speed = 0.45f;
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
            base_speed = 0.3f;
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
            base_speed = 0.3f;
            turn_speed = direction_PID(290.0f, yaw, gyro_z);
            if (current_step_dist >= 0.75) { // 累积里程判定
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
            base_speed = 0.2f;
            turn_speed = direction_PID(180.0f, yaw, gyro_z);
            if (current_step_dist >= 0.1) { // 累积里程判定
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
            turn_speed = direction_PID(90.0f, yaw, gyro_z);
            if (current_step_dist >= 0.9) { // 累积里程判定
                stop_car();
                current_running_task = TURN_OFF;
                Buzzer_and_LED(3);
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
 * @brief 发挥题（1）逻辑：舵机抬起并前进
 * @note 此函数应在定时中断中被持续调用
 */
extern float current_big_duty;
/**
 * @brief 发挥题（1）完整逻辑：吸球 -> 运球 0.2m -> 卸球
 * @note 运行在 20ms 定时中断中
 */
void task4_logic(void) {
    float base_speed = 0;
    float turn_speed = 0;
    static float step_start_dist = 0;
    static int wait_counter = 0;
    static int8_t expected_status = IMAGE_STATUS_ANY;
    ras_vision_result_t vision;
    bool has_vision = ras_get_latest_result(&vision);
    bool aligned = false;

    // 开发阶段：跳过前置动作，直接进入视觉对准
    if (sub_step < 3) {
        sub_step = 3;
    }

    switch (sub_step) {
        case 0: // 【已注释】前置动作调试时跳过
/*
            gpio_set_level(MEGNET_PIN, GPIO_HIGH);
            servo_set_target_up();
            image_control_reset();
            // 当前协议只有 0 和 -1，可先放开类别限制，避免对准阶段被卡住
            expected_status = IMAGE_STATUS_ANY;
            sub_step = 1;
*/
            break;

        case 1: // 【已注释】前置动作调试时跳过
/*
            if (abs(SERVO_MOTOR_R_MAX - current_big_duty) < ARRIVE_THRESHOLD) {
                step_start_dist = distance;
                sub_step = 2;
            }
*/
            break;

        case 2: // 【已注释】前置动作调试时跳过
/*
            base_speed = 0.15f;
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (distance - step_start_dist >= 0.30f) {
                stop_car();
                image_control_reset();
                sub_step = 3;
            }
*/
            break;

        case 3: // 【视觉闭环】对准球中心，死区+稳定计数防抖
            if (has_vision && image_control_update(&vision, expected_status, &base_speed, &turn_speed, &aligned)) {
                if (aligned) {
                    stop_car();
                    sub_step = 4;
                }
            } else {
                image_control_reset();
                base_speed = 0.0f;
                turn_speed = 0.0f;
            }
            break;

        case 4: // 【下降】准备吸球
            servo_set_target_down();
            sub_step = 5;
            break;

        case 5: // 【等待】舵机下降到位
            if (abs(SERVO_MOTOR_L_MAX - current_big_duty) < ARRIVE_THRESHOLD) {
                wait_counter = 0;
                sub_step = 6;
            }
            break;

        case 6: // 【吸球】稳定等待 400ms，确保电磁铁吸附
            // 20ms一次，20次即400ms，确保吸力稳定
            if (++wait_counter >= 20) {
                servo_set_target_up();
                sub_step = 7;
            }
            break;

        case 7: // 【等待】带球抬起到高位
            if (abs(SERVO_MOTOR_R_MAX - current_big_duty) < ARRIVE_THRESHOLD) {
                step_start_dist = distance; // 重要：重新记录起点，准备第二次前进
                sub_step = 8;
            }
            break;

        case 8: // 【运球】前进到投放区域附近
            base_speed = 0.15f; // 运球可以稍稳一点
            turn_speed = direction_PID(0.0f, yaw, gyro_z);
            if (distance - step_start_dist >= 0.15f) {
                stop_car();
                image_control_reset();
                // 当前协议仍返回 0，这里保持 ANY，后续有区分再改为 GOAL
                expected_status = IMAGE_STATUS_ANY;
                sub_step = 9;
            }
            break;

        case 9: // 【视觉闭环】对准投放目标中心
            if (has_vision && image_control_update(&vision, expected_status, &base_speed, &turn_speed, &aligned)) {
                if (aligned) {
                    stop_car();
                    sub_step = 10;
                }
            } else {
                image_control_reset();
                base_speed = 0.0f;
                turn_speed = 0.0f;
            }
            break;

        case 10: // 【下降】准备放球
            servo_set_target_down();
            sub_step = 11;
            break;

        case 11: // 【等待】舵机下降到位
            if (abs(SERVO_MOTOR_L_MAX - current_big_duty) < ARRIVE_THRESHOLD) {
                wait_counter = 0;
                sub_step = 12;
            }
            break;

        case 12: // 【释放】稳定后断开电磁铁
            if (++wait_counter >= 20) { // 等待400ms停稳
                gpio_set_level(MEGNET_PIN, GPIO_LOW); // 断开电磁铁
                wait_counter = 0;
                current_running_task = TURN_OFF;
            }
            break;
    }
    // 电机输出控制
    if (current_running_task != TURN_OFF) {
        // 统一限幅，避免视觉闭环或航向闭环导致速度过大
        base_speed = task4_clamp(base_speed, TASK4_MIN_BASE_SPEED, TASK4_MAX_BASE_SPEED);
        turn_speed = task4_clamp(turn_speed, -TASK4_MAX_TURN_SPEED, TASK4_MAX_TURN_SPEED);
        float left_target  = base_speed - turn_speed;
        float right_target = base_speed + turn_speed;
        small_driver_set_duty(speed_control_left_duty(left_target),
                                 speed_control_right_duty(right_target));
    }
}

