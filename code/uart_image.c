/*
 * uart_image.c
 *
 *  Created on: 2026年5月8日
 *      Author: cyz
 */

#include "zf_common_headfile.h"
#include "common.h"

// ================变量定义=======================


// 标签映射表 0..9, A..Z
static const char ras_char_map[36] = {
    '0','1','2','3','4','5','6','7','8','9',
    'A','B','C','D','E','F','G','H','I','J',
    'K','L','M','N','O','P','Q','R','S','T',
    'U','V','W','X','Y','Z'
};

// 接收行缓冲（中断写入）
static char ras_rx_line_buf[RAS_LINE_MAX];
static uint16_t ras_rx_line_idx = 0;
volatile static bool ras_rx_line_ready = false; // 中断设置，主循环读取

// 解析后存储（主循环写入/读取）
static ras_detection_t ras_detection_list[RAS_MAX_DETECTIONS];
static volatile uint16_t ras_detection_count = 0;

// 内部函数声明
static void ras_parse_line(const char *line);
static void ras_store_detection(const ras_detection_t *d);

// 公共函数实现

void ras_uart_init(void)
{
    ras_rx_line_idx = 0;
    ras_rx_line_ready = false;
    ras_clear_detections();
    memset(ras_rx_line_buf, 0, sizeof(ras_rx_line_buf));

    uart_init(RAS_UART, RAS_BAUDRATE, RAS_RX, RAS_TX);
    uart_rx_interrupt(RAS_UART, 1);
}

void ras_get_img(void)
{
    // 可选：向树莓派请求数据，例如：
    // uart_write_string(RAS_UART, "GET_IMG\n");
}

// 中断回调：只做字节读取与行组装，遇到换行设置就绪标志
void ras_uart_rx_callback(void)
{
    uint8_t receive_data;
    if(uart_query_byte(RAS_UART, &receive_data))
    {
        char ch = (char)receive_data;

        if(ch == '\r') return;

        if(ch == '\n')
        {
            if(ras_rx_line_idx > 0)
            {
                ras_rx_line_buf[ras_rx_line_idx] = '\0';
                ras_rx_line_ready = true; // 标志一行就绪
            }
            ras_rx_line_idx = 0;
            return;
        }

        if(ras_rx_line_idx < (RAS_LINE_MAX - 1))
        {
            ras_rx_line_buf[ras_rx_line_idx++] = ch;
        }
        else
        {
            // 溢出：重置并丢弃
            ras_rx_line_idx = 0;
            ras_rx_line_buf[0] = '\0';
        }
    }
}

// 主循环调用：若有就绪行则直接解析（无临界区保护）
void ras_uart_process(void)
{
    if(!ras_rx_line_ready) return;

    char line_copy[RAS_LINE_MAX];

    strcpy(line_copy, ras_rx_line_buf);

    ras_rx_line_ready = false;
    ras_rx_line_idx = 0;
    ras_rx_line_buf[0] = '\0';

    ras_parse_line(line_copy);
}

// 解析 CSV 行，格式： X,Y,LabelIndex,ResultChar
static void ras_parse_line(const char *line)
{
    if(line == NULL) return;
    if(line[0] == '\0') return;

    unsigned int x=0, y=0, label_idx=0;
    char result_char = 0;

    int parsed = sscanf(line, " %u , %u , %u , %c", &x, &y, &label_idx, &result_char);
    if(parsed != 4) return;
    if(label_idx > 35) return;

    ras_detection_t d;
    d.x = (uint16_t)x;
    d.y = (uint16_t)y;
    d.label_index = (uint8_t)label_idx;
    d.label_char = ras_char_map[label_idx];
    d.result_char = result_char;

    // 可选校验：若映射字符与 result 不一致，可记录或忽略
    ras_store_detection(&d);
}

// 存储解析结果
static void ras_store_detection(const ras_detection_t *d)
{
    if(d == NULL) return;

    ras_detection_list[0] = *d;
    ras_detection_count = 1;
}

void ras_clear_detections(void)
{
    ras_detection_count = 0;
    memset(ras_detection_list, 0, sizeof(ras_detection_list));
}

uint16_t ras_get_detection_count(void)
{
    return ras_detection_count;
}

bool ras_get_detection(uint16_t idx, ras_detection_t *out)
{
    if(out == NULL) return false;
    if(idx >= ras_detection_count) return false;
    *out = ras_detection_list[idx];
    return true;
}



