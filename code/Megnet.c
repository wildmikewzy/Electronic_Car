/*
 * magnet.c
 *
 *  Created on: 2026年5月10日
 *      Author: Wild_Mike_wzy
 */
#include "zf_common_headfile.h"
/**
 * @brief 电磁铁初始化
 */
void Magnet_init(void){
    gpio_init(MEGNET_PIN, GPO, 0, GPO_PUSH_PULL);
}
/**
 * @brief 电磁铁吸附
 */
void Magnet_absorb(void){
    gpio_set_level(MEGNET_PIN,1);
}
/**
 * @brief  电磁铁释放
 */
void Magnet_release(void){
    gpio_set_level(MEGNET_PIN,0);
}


