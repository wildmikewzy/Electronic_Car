/*
 * image.h
 *
 *  Created on: 2026-05-14
 *      Author: cyz
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"
#include "common.h"
#include "uart_image.h"
#include <stdbool.h>
#include <stdint.h>

// 图像尺寸与目标中心
#define IMAGE_FRAME_WIDTH      640
#define IMAGE_FRAME_HEIGHT     480
#define IMAGE_TARGET_CENTER_X  330
#define IMAGE_TARGET_CENTER_Y  330
#define IMAGE_TARGET_CENTER_Y_2  280
#define IMAGE_TARGET_CENTER_Y_BUCKET (250)

// 视觉类别定义（与上位机/树莓派协议保持一致）
#define IMAGE_STATUS_BALL      0
#define IMAGE_STATUS_GOAL      1
#define IMAGE_STATUS_ANY       (-2)

// 视觉闭环初始化与重置（清空PID状态与稳定计数）
void image_control_init(void);
void image_control_reset(void);
// 视觉闭环更新-y轴
bool image_control_update(const ras_vision_result_t *result,
                          int8_t expected_status,
                          float *base_speed,
                          float *turn_speed,
                          bool *aligned,
                          float image_target_y);
//视觉更新-x轴
bool image_control_update_x(const ras_vision_result_t *result,
                            float *turn_speed);
// 获取最近一次视觉闭环的调试数据（不触发计算）
bool image_get_debug(float *base_speed,
                     float *turn_speed,
                     int16_t *err_x,
                     int16_t *err_y,
                     bool *aligned,
                     bool *valid);

#endif /* CODE_IMAGE_H_ */

