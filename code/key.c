#include "zf_common_headfile.h"
// --- 全局变量声明 (外部定义的 VOFA 基准值) ---

void Key_init(void){
    gpio_init(KEY_UP, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY_LEFT, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY_RIGHT, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
    gpio_init(KEY_LEFT, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
}
/**
 *@brief：按键状态检测函数
 */
uint8 GetKey_UP(void){
    return !gpio_get_level(KEY_UP);
}

uint8 GetKey_DOWN(void){
    return !gpio_get_level(KEY_DOWN);
}

uint8 GetKey_LEFT(void){
    return !gpio_get_level(KEY_LEFT);
}

uint8 GetKey_RIGHT(void){
    return !gpio_get_level(KEY_RIGHT);
}
uint8 GetKey_ENTER(void){
    return !gpio_get_level(KEY_ENTER);
}
