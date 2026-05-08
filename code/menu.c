/*
 * menu.c
 *
 *  Created on: 2026年5月3日
 *      Author: Wild_Mike_wzy
 */
#include "menu.h"
extern float speed;
extern float distance;
extern Pose_t car_pose;
/**
 * @brief 屏幕初始化6
 */
void menu_init(void){
    ips114_set_dir(IPS114_PORTAIT_180);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
}
/**
 * @brief 屏幕现实函数
 */
void screen_update(void){
    ips114_show_string(5, 16,    "PIT:");
    ips114_show_string(5, 16*2,  "ROL:");
    ips114_show_string(5, 16*3,  "YAW:");
    ips114_show_string(5, 16*4,  "speed:");
    ips114_show_string(5,  16*5,  "distance:");
    ips114_show_string(5,  16*6, "POS X:");
    ips114_show_string(5,  16*7, "POS Y:");


    ips114_show_float(45, 16, pitch, 3, 1);
    ips114_show_float(45, 16*2, roll,  3, 1);
    ips114_show_float(45, 16*3, yaw,   3, 1);
    ips114_show_float(8*7, 16*4,speed, 3, 3);
    ips114_show_float(8*10, 16*5,distance, 5, 3);
    ips114_show_float(5+8*6, 16*6,  car_pose.x, 3, 2);
    ips114_show_float(5+8*6, 16*7, car_pose.y, 3, 2);

}

