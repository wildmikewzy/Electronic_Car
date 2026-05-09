/*
 * gray.c
 *
 *  Created on: 2026年5月9日
 *      Author: cyz
 */


#include "zf_common_headfile.h"
#include "common.h"


static volatile uint8_t gray_level[GRAY_SENSOR_NUM] = {0};
static volatile uint8_t gray_active[GRAY_SENSOR_NUM] = {0};

static volatile uint8_t gray_mask = 0;
static volatile int16_t gray_error = 0;
static volatile int16_t gray_last_error = 0;
static volatile uint8_t gray_valid_count = 0;
static volatile bool gray_lost_flag = true;

// 权重：从左到右
// GRAY_1 最左，GRAY_8 最右
static const int8_t gray_weight[GRAY_SENSOR_NUM] =
{
    -4, -3, -2, -1, 1, 2, 3, 4
};

void gray_init(void)
{
    gpio_init(GRAY_1, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_2, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_3, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_4, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_5, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_6, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_7, GPI, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_8, GPI, GPIO_LOW, GPO_PUSH_PULL);

    for(uint8_t i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        gray_level[i] = 0;
        gray_active[i] = 0;
    }

    gray_mask = 0;
    gray_error = 0;
    gray_last_error = 0;
    gray_valid_count = 0;
    gray_lost_flag = true;
}

void gray_update(void)
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    uint8_t mask = 0;

    gray_level[0] = gpio_get_level(GRAY_1);
    gray_level[1] = gpio_get_level(GRAY_2);
    gray_level[2] = gpio_get_level(GRAY_3);
    gray_level[3] = gpio_get_level(GRAY_4);
    gray_level[4] = gpio_get_level(GRAY_5);
    gray_level[5] = gpio_get_level(GRAY_6);
    gray_level[6] = gpio_get_level(GRAY_7);
    gray_level[7] = gpio_get_level(GRAY_8);

    for(uint8_t i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        if(gray_level[i] == GRAY_ACTIVE_LEVEL)
        {
            gray_active[i] = 1;
            mask |= (uint8_t)(1 << i);

            weighted_sum += gray_weight[i];
            active_count++;
        }
        else
        {
            gray_active[i] = 0;
        }
    }

    gray_mask = mask;
    gray_valid_count = active_count;

    if(active_count > 0)
    {
        gray_lost_flag = false;

        gray_error = weighted_sum / active_count;
        gray_last_error = gray_error;
    }
    else
    {
        gray_lost_flag = true;

#if GRAY_LOST_KEEP_LAST
        gray_error = gray_last_error;
#else
        gray_error = 0;
#endif
    }
}

int16_t gray_get_error(void)
{
    return gray_error;
}

int16_t gray_get_last_error(void)
{
    return gray_last_error;
}

uint8_t gray_get_mask(void)
{
    return gray_mask;
}

uint8_t gray_get_valid_count(void)
{
    return gray_valid_count;
}

bool gray_is_lost(void)
{
    return gray_lost_flag;
}

uint8_t gray_get_level_value(uint8_t index)
{
    if(index >= GRAY_SENSOR_NUM)
    {
        return 0;
    }

    return gray_level[index];
}

uint8_t gray_get_active_value(uint8_t index)
{
    if(index >= GRAY_SENSOR_NUM)
    {
        return 0;
    }

    return gray_active[index];
}
