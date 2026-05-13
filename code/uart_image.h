
/*
 * uart_image.h
 *
 *  Created on: 2026年5月8日
 *      Author: cyz
 */

#ifndef CODE_UART_IMAGE_H_
#define CODE_UART_IMAGE_H_

#include "zf_common_headfile.h"
#include "common.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef RAS_LINE_MAX
#define RAS_LINE_MAX        32
#endif

// 视觉检测结果
typedef struct
{
    int8_t   status;      // -1: 未检测到, 0: 第一类目标, 1: 第二类目标
    uint16_t x;           // 横坐标
    uint16_t y;           // 纵坐标
} ras_vision_result_t;

// 初始化串口
void ras_uart_init(void);

// 可选：向树莓派请求数据
void ras_get_img(void);

// UART RX 中断中调用
void ras_uart_rx_callback(void);

// 主循环或定时器中调用，解析完整一行
void ras_uart_process(void);

// 清除当前结果
void ras_clear_result(void);

// 获取最新结果，不清除新数据标志
bool ras_get_latest_result(ras_vision_result_t *out);

// 获取新结果，读取后清除新数据标志
bool ras_get_new_result(ras_vision_result_t *out);

// 是否已经收到过有效结果
bool ras_has_result(void);

// 是否有新结果
bool ras_has_new_result(void);

#endif /* CODE_UART_IMAGE_H_ */


