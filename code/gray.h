/*
 * gray.h
 *
 *  Created on: 2026年5月9日
 *      Author: cyz
 */

#ifndef CODE_GRAY_H_
#define CODE_GRAY_H_

#include "zf_common_headfile.h"
#include "common.h"

#define GRAY_SENSOR_NUM        8

// 若灰度传感器检测到黑线时输出高电平，改成 GPIO_HIGH
#define GRAY_ACTIVE_LEVEL      GPIO_LOW

// 丢线时是否保持上一次 error
#define GRAY_LOST_KEEP_LAST    1

void gray_init(void);
void gray_update(void);

int16_t gray_get_error(void);
int16_t gray_get_last_error(void);
uint8_t gray_get_mask(void);
uint8_t gray_get_valid_count(void);
bool gray_is_lost(void);

uint8_t gray_get_level_value(uint8_t index);
uint8_t gray_get_active_value(uint8_t index);

#endif /* CODE_GRAY_H_ */
