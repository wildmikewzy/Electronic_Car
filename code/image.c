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
#define IMAGE_DEADBAND_X       8
#define IMAGE_DEADBAND_Y       8
#define IMAGE_ALIGN_COUNT      6

// 视觉PID参数：X控制转向，Y控制前后速度
#define IMAGE_PID_X_KP         0.008f
#define IMAGE_PID_X_KI         0.0f
#define IMAGE_PID_X_KD         0.002f
#define IMAGE_PID_Y_KP         0.0015f
#define IMAGE_PID_Y_KI         0.0f
#define IMAGE_PID_Y_KD         0.0005f

// 输出限幅：保护速度环与底盘稳定性
#define IMAGE_MAX_TURN_SPEED   0.6f
#define IMAGE_MAX_BASE_SPEED   0.25f
#define IMAGE_MIN_BASE_SPEED  -0.15f

// X轴用于转向闭环，Y轴用于前后闭环
static PID_t image_pid_x;
static PID_t image_pid_y;
static uint16_t image_target_x = IMAGE_TARGET_CENTER_X;
static uint16_t image_target_y = IMAGE_TARGET_CENTER_Y;
static uint8_t image_stable_count = 0;

// 最近一次计算结果，便于在中断中打印调试
static float image_last_base_speed = 0.0f;
static float image_last_turn_speed = 0.0f;
static int16_t image_last_err_x = 0;
static int16_t image_last_err_y = 0;
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

// 增量式PID计算（与速度环一致的思路，输出为float）
static float image_pid_compute(PID_t *pid,
                               float measure_val,
                               float output_min,
                               float output_max,
                               bool hold_zero)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    // 进入死区时直接清零，抑制抖动
    if(hold_zero)
    {
        image_pid_reset(pid);
        return 0.0f;
    }

    pid->err = pid->target_val - measure_val;

    float increment = pid->kp * (pid->err - pid->last_err) +
                      pid->ki * pid->err +
                      pid->kd * (pid->err - 2 * pid->last_err + pid->prev_err);

    pid->output_f += increment;
    pid->output_f = image_clamp(pid->output_f, output_min, output_max);

    pid->prev_err = pid->last_err;
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

// 视觉闭环更新：返回是否有效识别，并输出速度与对准标志
bool image_control_update(const ras_vision_result_t *result,
                          int8_t expected_status,
                          float *base_speed,
                          float *turn_speed,
                          bool *aligned)
{
    if(base_speed != NULL)
    {
        *base_speed = 0.0f;
    }
    if(turn_speed != NULL)
    {
        *turn_speed = 0.0f;
    }
    if(aligned != NULL)
    {
        *aligned = false;
    }

    if(result == NULL || base_speed == NULL || turn_speed == NULL || aligned == NULL)
    {
        image_control_reset();
        return false;
    }

    // 未检测到目标
    if(result->status < 0)
    {
        image_control_reset();
        return false;
    }

    // 目标类别不匹配时拒绝进入闭环
    if(expected_status != IMAGE_STATUS_ANY && result->status != expected_status)
    {
        image_control_reset();
        return false;
    }

    // 坐标越界时直接丢弃
    if(result->x >= IMAGE_FRAME_WIDTH || result->y >= IMAGE_FRAME_HEIGHT)
    {
        image_control_reset();
        return false;
    }

    float err_x = (float)image_target_x - (float)result->x;
    float err_y = (float)image_target_y - (float)result->y;

    // 死区判断：小范围误差不驱动
    bool in_x = (fabsf(err_x) <= IMAGE_DEADBAND_X);
    bool in_y = (fabsf(err_y) <= IMAGE_DEADBAND_Y);

    // 稳定计数：连续满足死区才判定对准
    if(in_x && in_y)
    {
        if(image_stable_count < 255)
        {
            image_stable_count++;
        }
    }
    else
    {
        image_stable_count = 0;
    }

    image_pid_x.target_val = (float)image_target_x;
    image_pid_y.target_val = (float)image_target_y;

    // X轴控制转向，Y轴控制前后
    *turn_speed = image_pid_compute(&image_pid_x,
                                    (float)result->x,
                                    -IMAGE_MAX_TURN_SPEED,
                                    IMAGE_MAX_TURN_SPEED,
                                    in_x);

    *base_speed = image_pid_compute(&image_pid_y,
                                    (float)result->y,
                                    IMAGE_MIN_BASE_SPEED,
                                    IMAGE_MAX_BASE_SPEED,
                                    in_y);

    if(image_stable_count >= IMAGE_ALIGN_COUNT)
    {
        *aligned = true;
    }

    // 保存调试数据，方便在中断里打印
    image_last_base_speed = *base_speed;
    image_last_turn_speed = *turn_speed;
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
