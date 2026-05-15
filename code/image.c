/*
 * image.c
 *
 *  Created on: 2026-05-14
 *      Author: cyz
 */

#include "zf_common_headfile.h"
#include "common.h"
#include "uart_image.h"
// 死区与稳定计数：避免轻微抖动导致反复调整
#define IMAGE_DEADBAND_X       20
#define IMAGE_DEADBAND_Y       20
#define IMAGE_ALIGN_COUNT      6

// 视觉PID参数：X控制转向，Y控制前后速度
#define IMAGE_PID_X_KP         0.0000f
#define IMAGE_PID_X_KI         0.000f
#define IMAGE_PID_X_KD         0.0f
#define IMAGE_PID_Y_KP         0.0005f
#define IMAGE_PID_Y_KI         0.001f
#define IMAGE_PID_Y_KD         0.2f

// 输出限幅：保护速度环与底盘稳定性
#define IMAGE_MAX_TURN_SPEED   0.05f
#define IMAGE_MAX_BASE_SPEED   0.2f
#define IMAGE_MIN_BASE_SPEED  -0.15f

// X轴用于转向闭环，Y轴用于前后闭环
static PID_t image_pid_x;
static PID_t image_pid_y;
static uint16_t image_target_x = IMAGE_TARGET_CENTER_X;
static uint16_t image_target_y = IMAGE_TARGET_CENTER_Y;

//static uint16_t image_target_y_bucket = IMAGE_TARGET_CENTER_Y_BUCKET;
static uint8_t image_stable_count = 0;

// 最近一次计算结果，便于在中断中打印调试
static float image_last_base_speed = 0.0f;
static float image_last_turn_speed = 0.0f;
int16_t image_last_err_x = 0;
int16_t image_last_err_y = 0;
static bool image_last_aligned = false;
static bool image_last_valid = false;

// 简单限幅函数
static float image_clamp(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

// 清空PID中间状态，避免上一次残留导致突跳
static void image_pid_reset(PID_t *pid)
{
    if(pid == NULL)
    {
        return;
    }

    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->output_f = 0.0f;
    pid->output = 0;
}

// 修改后的计算函数：采用位置式思路 + 限制积分幅度
static float image_pid_compute(PID_t *pid,
                               float measure_val,
                               float output_min,
                               float output_max,
                               bool in_deadband)
{
    if(pid == NULL) return 0.0f;

    // 1. 如果在死区内，直接清空状态并返回 0
    if(in_deadband)
    {
        pid->err = 0;
        pid->last_err = 0;
        pid->output_f = 0; // 位置式直接归零
        return 0.0f;
    }

    pid->err = pid->target_val - measure_val;

    // 2. 位置式 PID 计算
    // P 项
    float p_out = pid->kp * pid->err;

    // D 项 (视觉信号抖动大，建议 KD 设为 0 或极小)
    float d_out = pid->kd * (pid->err - pid->last_err);

    // 3. 计算基础输出
    pid->output_f = p_out + d_out;

    // 4. 【关键】静摩擦力补偿 (死区补偿)
    // 假设电机 0.03f 才能转动，我们给它一个基础推力
    const float MIN_DRIVE = 0.035f;
    if (pid->output_f > 0) pid->output_f += MIN_DRIVE;
    else if (pid->output_f < 0) pid->output_f -= MIN_DRIVE;

    // 5. 限幅输出
    pid->output_f = image_clamp(pid->output_f, output_min, output_max);

    pid->last_err = pid->err;

    return pid->output_f;
}

// 初始化视觉闭环参数
void image_control_init(void)
{
    image_pid_x.kp = IMAGE_PID_X_KP;
    image_pid_x.ki = IMAGE_PID_X_KI;
    image_pid_x.kd = IMAGE_PID_X_KD;
    image_pid_x.target_val = (float)image_target_x;

    image_pid_y.kp = IMAGE_PID_Y_KP;
    image_pid_y.ki = IMAGE_PID_Y_KI;
    image_pid_y.kd = IMAGE_PID_Y_KD;
    image_pid_y.target_val = (float)image_target_y;

    image_control_reset();
}

// 重置视觉闭环状态（中断切换/丢失目标时调用）
void image_control_reset(void)
{
    image_pid_reset(&image_pid_x);
    image_pid_reset(&image_pid_y);
    image_stable_count = 0;
    image_last_base_speed = 0.0f;
    image_last_turn_speed = 0.0f;
    image_last_err_x = 0;
    image_last_err_y = 0;
    image_last_aligned = false;
    image_last_valid = false;
}
/**
 * @brief 视觉控制：仅纵向闭环（手动对准 X 轴版）
 * 逻辑：假设 X 轴已经准了，视觉只负责把车开到球面前并停稳。
 */
bool image_control_update(const ras_vision_result_t *result,
                          int8_t expected_status,
                          float *base_speed,
                          float *turn_speed,
                          bool *aligned,
                          float image_target_y)
{
    // --- 1. 基础检查 ---
    if(result == NULL || base_speed == NULL || turn_speed == NULL || aligned == NULL) {
        image_control_reset();
        return false;
    }
    *base_speed = 0.0f;
    *turn_speed = 0.0f; // 彻底放弃 X 轴控制
    *aligned = false;

    // 目标检查
    if(result->status < 0 || result->x >= IMAGE_FRAME_WIDTH || result->y >= IMAGE_FRAME_HEIGHT) {
        image_control_reset();
        return false;
    }

    // --- 2. 偏差计算与非线性压缩 ---
    float err_y = (float)image_target_y - (float)result->y;
    float err_x = (float)image_target_x - (float)result->x; // 仅用于判定是否“真的准了”

    float processed_y = (float)result->y;

    // Y 轴大偏差压缩：当距离球较远时（err_y > 50），压缩输出，防止猛冲
    if (fabsf(err_y) > 50.0f) {
        processed_y = (float)image_target_y - (err_y * 0.5f);
    }

    // --- 3. 闭环 Y 轴计算 ---
    bool in_y_deadband = (fabsf(err_y) <= IMAGE_DEADBAND_Y);
    //根据 err_y 动态调整速度限幅
    float dynamic_max_speed;
    if (err_y > 100) {
        dynamic_max_speed = 0.20f; // 离得远，跑快点
    } else {
        dynamic_max_speed = 0.06f; // 靠近了，切回慢速模式确保精度
    }

    *base_speed = image_pid_compute(&image_pid_y, processed_y,
                                    IMAGE_MIN_BASE_SPEED, dynamic_max_speed, in_y_deadband);


    // --- 4. 判定是否到达抓取点 ---
    // 条件：Y 进入死区（y位置准了）
    if (in_y_deadband) {
        if (image_stable_count < 255) image_stable_count++;
    } else {
        image_stable_count = 0;
    }

    // 计数达到阈值判定为对准（建议设大一点，确保车彻底停稳）
    if (image_stable_count >= IMAGE_ALIGN_COUNT) {
        *aligned = true;
    }

    // --- 5. 调试数据保存 ---
    image_last_base_speed = *base_speed;
    image_last_turn_speed = 0.0f;
    image_last_err_x = (int16_t)err_x;
    image_last_err_y = (int16_t)err_y;
    image_last_aligned = *aligned;
    image_last_valid = true;

    return true;
}
// 获取最近一次视觉闭环调试数据
bool image_get_debug(float *base_speed,
                     float *turn_speed,
                     int16_t *err_x,
                     int16_t *err_y,
                     bool *aligned,
                     bool *valid)
{
    if(base_speed == NULL || turn_speed == NULL || err_x == NULL || err_y == NULL || aligned == NULL || valid == NULL)
    {
        return false;
    }

    *base_speed = image_last_base_speed;
    *turn_speed = image_last_turn_speed;
    *err_x = image_last_err_x;
    *err_y = image_last_err_y;
    *aligned = image_last_aligned;
    *valid = image_last_valid;

    return true;
}
