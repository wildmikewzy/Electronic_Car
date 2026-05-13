/*
 * uart_image.c
 *
 *  Created on: 2026年5月8日
 *      Author: cyz
 */

#include "uart_image.h"

// ==================================================
// 串口接收缓冲
// ==================================================

// UART 中断正在写入的缓冲
static char ras_rx_build_buf[RAS_LINE_MAX];
static uint16_t ras_rx_build_idx = 0;

// 已经接收完成、等待解析的缓冲
static char ras_rx_ready_buf[RAS_LINE_MAX];
static volatile bool ras_rx_line_ready = false;

// ==================================================
// 最新视觉结果
// ==================================================

static ras_vision_result_t ras_latest_result;
static volatile bool ras_result_valid = false;
static volatile bool ras_new_result_flag = false;

// ==================================================
// 内部函数声明
// ==================================================

static void ras_parse_line(const char *line);
static void ras_store_result(const ras_vision_result_t *result);

static void ras_skip_space(const char **p);
static bool ras_parse_i32(const char **p, int32_t *out);
static bool ras_expect_comma(const char **p);

static void ras_str_copy(char *dst, const char *src, uint16_t max_len);

// ==================================================
// 初始化
// ==================================================

void ras_uart_init(void)
{
    ras_rx_build_idx = 0;
    ras_rx_line_ready = false;

    for(uint16_t i = 0; i < RAS_LINE_MAX; i++)
    {
        ras_rx_build_buf[i] = '\0';
        ras_rx_ready_buf[i] = '\0';
    }

    ras_clear_result();

    uart_init(RAS_UART, RAS_BAUDRATE, RAS_RX, RAS_TX);
    uart_rx_interrupt(RAS_UART, 1);
}

void ras_get_img(void)
{
    // 如果树莓派需要主动请求，可以打开这一句
    // uart_write_string(RAS_UART, "GET_IMG\n");
}

// ==================================================
// UART RX 中断回调
// 功能：只接收字节并拼接一行
// 协议格式：status,x,y\n
// 示例：-1,0,0
// 示例：0,266,149
// 示例：1,266,149
// ==================================================

void ras_uart_rx_callback(void)
{
    uint8_t receive_data;

    while(uart_query_byte(RAS_UART, &receive_data))
    {
        char ch = (char)receive_data;

        if(ch == '\r')
        {
            continue;
        }

        if(ch == '\n')
        {
            if(ras_rx_build_idx > 0)
            {
                ras_rx_build_buf[ras_rx_build_idx] = '\0';

                // 保存完整一行
                // 如果上一行还没处理，这里直接覆盖为最新数据
                ras_str_copy(ras_rx_ready_buf, ras_rx_build_buf, RAS_LINE_MAX);

                ras_rx_line_ready = true;
            }

            ras_rx_build_idx = 0;
            ras_rx_build_buf[0] = '\0';

            continue;
        }

        if(ras_rx_build_idx < RAS_LINE_MAX - 1)
        {
            ras_rx_build_buf[ras_rx_build_idx++] = ch;
        }
        else
        {
            // 行太长，丢弃当前行
            ras_rx_build_idx = 0;
            ras_rx_build_buf[0] = '\0';
        }
    }
}

// ==================================================
// 处理完整行
// 建议放在主循环里调用
// 你现在放在 PIT 中断里也能用，但不建议在中断里 printf 太频繁
// ==================================================

void ras_uart_process(void)
{
    if(!ras_rx_line_ready)
    {
        return;
    }

    char line_copy[RAS_LINE_MAX];

    ras_str_copy(line_copy, ras_rx_ready_buf, RAS_LINE_MAX);

    ras_rx_line_ready = false;

    ras_parse_line(line_copy);
}

// ==================================================
// 解析一行数据
// 协议：status,x,y
// 识别到：0,266,149 或 1,266,149
// 未识别：-1,-1,-1
// ==================================================

static void ras_parse_line(const char *line)
{
    if(line == NULL || line[0] == '\0')
    {
        return;
    }

    const char *p = line;
    int32_t status = 0;
    int32_t x = 0;
    int32_t y = 0;

    // 1. 解析三个整型参数
    if(!ras_parse_i32(&p, &status)) return;
    if(!ras_expect_comma(&p))        return;
    if(!ras_parse_i32(&p, &x))      return;
    if(!ras_expect_comma(&p))        return;
    if(!ras_parse_i32(&p, &y))      return;

    ras_skip_space(&p);
    if(*p != '\0') return; // 后面不能有多余字符

    // 2. 逻辑判断与校验
    ras_vision_result_t result;

    if(status == -1)
    {
        // 情况 A: 树莓派发送 -1,-1,-1 (未检测到)
        result.status = -1;
        result.x = 0; // 即使收到-1，MCU内部也记录为0，方便控制逻辑判断
        result.y = 0;
    }
    else if(status == 0 || status == 1)
    {
        // 情况 B: 检测到目标，校验坐标合法性
        if(x < 0 || y < 0 || x > 65535 || y > 65535)
        {
            return; // 坐标范围非法
        }
        result.status = (int8_t)status;
        result.x = (uint16_t)x;
        result.y = (uint16_t)y;
    }
    else
    {
        // 情况 C: 未知的 status 状态
        return;
    }

    // 3. 存储解析后的结果
    ras_store_result(&result);
}

// ==================================================
// 保存最新结果
// ==================================================

static void ras_store_result(const ras_vision_result_t *result)
{
    if(result == NULL)
    {
        return;
    }

    ras_latest_result = *result;
    ras_result_valid = true;
    ras_new_result_flag = true;
}

// ==================================================
// 对外访问接口
// ==================================================

void ras_clear_result(void)
{
    ras_latest_result.status = -1;
    ras_latest_result.x = 0;
    ras_latest_result.y = 0;

    ras_result_valid = false;
    ras_new_result_flag = false;
}

bool ras_get_latest_result(ras_vision_result_t *out)
{
    if(out == NULL)
    {
        return false;
    }

    if(!ras_result_valid)
    {
        return false;
    }

    *out = ras_latest_result;
    return true;
}

bool ras_get_new_result(ras_vision_result_t *out)
{
    if(out == NULL)
    {
        return false;
    }

    if(!ras_result_valid)
    {
        return false;
    }

    if(!ras_new_result_flag)
    {
        return false;
    }

    *out = ras_latest_result;

    // 取走后清除新数据标志，避免一直重复打印旧数据
    ras_new_result_flag = false;

    return true;
}

bool ras_has_result(void)
{
    return ras_result_valid;
}

bool ras_has_new_result(void)
{
    return ras_new_result_flag;
}

// ==================================================
// 工具函数：跳过空格
// ==================================================

static void ras_skip_space(const char **p)
{
    if(p == NULL)
    {
        return;
    }

    while(**p == ' ' || **p == '\t')
    {
        (*p)++;
    }
}

// ==================================================
// 工具函数：解析有符号整数
// 支持 -1、0、1、266 这类数字
// ==================================================

static bool ras_parse_i32(const char **p, int32_t *out)
{
    int32_t value = 0;
    int8_t sign = 1;
    bool has_digit = false;

    if(p == NULL || out == NULL)
    {
        return false;
    }

    ras_skip_space(p);

    if(**p == '-')
    {
        sign = -1;
        (*p)++;
    }
    else if(**p == '+')
    {
        (*p)++;
    }

    while(**p >= '0' && **p <= '9')
    {
        has_digit = true;
        value = value * 10 + (int32_t)(**p - '0');
        (*p)++;
    }

    if(!has_digit)
    {
        return false;
    }

    ras_skip_space(p);

    *out = value * sign;
    return true;
}

// ==================================================
// 工具函数：匹配逗号
// ==================================================

static bool ras_expect_comma(const char **p)
{
    if(p == NULL)
    {
        return false;
    }

    ras_skip_space(p);

    if(**p != ',')
    {
        return false;
    }

    (*p)++;

    ras_skip_space(p);

    return true;
}

// ==================================================
// 工具函数：安全字符串复制
// 不用 strcpy，避免缓冲区越界
// ==================================================

static void ras_str_copy(char *dst, const char *src, uint16_t max_len)
{
    uint16_t i = 0;

    if(dst == NULL || src == NULL || max_len == 0)
    {
        return;
    }

    while(i < max_len - 1 && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }

    dst[i] = '\0';
}
