/*
 * menu.h
 *
 *  Created on: 2026年5月3日
 *      Author: 22619
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"
//=====================================结构体定义=======================================
 /**
  *@brief 主菜单显示项结构体
  */
 typedef struct {
     const char *name;   // 菜单项显示的英文字模
     int id;                  // 选中该项后返回给 main 函数的 ID (即 menu2_flag)
 } MainMenuItem;
 typedef struct {
     uint16 x, y;
     const char *content;    // 内容
     void (*action)(void);   // 触发动作（可选，用于解耦清零逻辑）
 } MenuActionItem;
//===========================函数声明===================================
void menu_init(void);
int menu1(void);
void screen_update(void);
void menu2_RealTimeDisplay(void);
#endif /* CODE_MENU_H_ */
