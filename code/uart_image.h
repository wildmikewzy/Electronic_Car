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


#ifndef RAS_LINE_MAX
#define RAS_LINE_MAX        128
#endif

#ifndef RAS_MAX_DETECTIONS
#define RAS_MAX_DETECTIONS  32
#endif

// 检测目标结构体
typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t  label_index;
    char     label_char;
    char     result_char;
} ras_detection_t;

// 初始化与入口
void ras_uart_init(void);
void ras_get_img(void);

// 在 UART RX 中断中调用（只读字节并组行）
void ras_uart_rx_callback(void);

// 主循环调用：处理就绪行并解析
void ras_uart_process(void);

// 访问接口
uint16_t ras_get_detection_count(void);
bool ras_get_detection(uint16_t idx, ras_detection_t *out);
void ras_clear_detections(void);



#endif /* CODE_UART_IMAGE_H_ */
