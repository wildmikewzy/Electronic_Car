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
//===========================================菜单选项登记表=================================================
 // ==================== 一级菜单总表 ====================
 const MainMenuItem MainMenuEntries[] = {
     {"RealTimeDisplay",1},   //实时数据显示
     {"Base1", 2}, // 1. 基础科目（1）
     {"Base2", 3}, // 2. 基础科目（2）
     {"Base3", 4}, // 3. 基础科目（3）
     {"Advance1",5},     //4.发挥题目（1）
     {"Advance2",6},     //5.发挥题目（2）
     {"Advance3",7},     //6.发挥题目（3）
 };
#define MAIN_MENU_NUM (sizeof(MainMenuEntries) / sizeof(MainMenuItem))      //主菜单选项数量
#define MAX_VISIBLE_ROWS 4  // 屏幕一页最多显示的行数
 // ==================== 动作函数定义 ====================
/**
 * @brief 重置清零
 */
 void action_reset_data(void) {
     car_pose.x = 0;
     car_pose.y = 0;
     ekf_reset();
 }
 // ==================== 交互项登记 ====================
 // 这里只登记“可以被选中”的项：返回键 和 清零键
 const MenuActionItem RealTimeMenuItems[] = {
     {0,   0,  "<<-----", NULL},               // Index 1: 返回
     {170, 16*7,"CLEAR", action_reset_data}   // Index 2: 清零
 };
 #define RT_MENU_NUM (sizeof(RealTimeMenuItems) / sizeof(MenuActionItem))

/**
 * @brief 屏幕初始化
 */
void menu_init(void){
    ips114_set_dir(IPS114_PORTAIT_180);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
}
/**
 *@brief 支持窗口滚动的渲染函数
 *@param cursor 当前光标所在的绝对序号 (1 ~ MAIN_MENU_NUM)
 */
void menu1_render(int cursor, int selected_id) {
    ips114_show_string(32*1+16, 0, "Electronic Car");

    // 计算当前窗口的起始索引 (0-indexed)
    // 逻辑：确保 cursor 始终在 [window_start + 1, window_start + MAX_VISIBLE_ROWS] 范围内
    static int window_start = 0;

    if (cursor - 1 < window_start) {
        window_start = cursor - 1;
    } else if (cursor - 1 >= window_start + MAX_VISIBLE_ROWS) {
        window_start = cursor - MAX_VISIBLE_ROWS;
    }

    for (int i = 0; i < MAX_VISIBLE_ROWS; i++) {
        int item_idx = window_start + i;
        if (item_idx >= MAIN_MENU_NUM) break; // 超过总数则停止绘制

        // 1. 处理高亮（正在滑动的光标）
        if (item_idx == (cursor - 1)) {
            ips114_set_color(RGB565_WHITE, RGB565_BLUE);
        } else {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        }

        // 绘制菜单名，y轴偏移基于 i (相对窗口的位置) 而非 item_idx
        ips114_show_string(0, 40 + (i * 24), MainMenuEntries[item_idx].name);

        // 2. 处理 "selected" 标志
        if (MainMenuEntries[item_idx].id == selected_id && selected_id > 1) {
            ips114_set_color(RGB565_GREEN, RGB565_BLACK);
            ips114_show_string(8*12, 40 + (i * 24), "SEL"); // 缩短字样防止重叠
        }
    }

//    // 绘制滚动条提示（可选，增加交互感）
//    if (MAIN_MENU_NUM > MAX_VISIBLE_ROWS) {
//        ips114_set_color(RGB565_GRAY, RGB565_BLACK);
//        ips114_draw_line(235, 40, 235, 40 + (MAX_VISIBLE_ROWS * 24), RGB565_GRAY);
//        // 简单滑块逻辑
//        int slider_y = 40 + ( (cursor-1) * (MAX_VISIBLE_ROWS * 24) / MAIN_MENU_NUM );
//        ips114_draw_line(234, slider_y, 236, slider_y + 10, RGB565_WHITE);
//    }

    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
}
// 定义一个全局变量记录当前选中的科目 ID
int current_selected_task = 0;
/*
 * @ brief:一级菜单显示函数
 * @ parameter ：None
 * @ return value ：flag，根据不同的flag数值进入相应的二级菜单
 * */

int menu1(void) {
    static int cursor = 1; // 光标位置
    uint8 update_needed = 1;

    while(true) {
        if(update_needed) {
            ips114_clear();
            menu1_render(cursor, current_selected_task);
            update_needed = 0;
        }

        if(GetKey_UP()) {
            if(--cursor < 1) cursor = MAIN_MENU_NUM;
            update_needed = 1;
            system_delay_ms(200);
        }
        if(GetKey_DOWN()) {
            if(++cursor > MAIN_MENU_NUM) cursor = 1;
            update_needed = 1;
            system_delay_ms(200);
        }

        if(GetKey_ENTER()) {
            system_delay_ms(200);
            int selected_id = MainMenuEntries[cursor - 1].id;

            if(selected_id == 1) {
                // 如果是实时显示，直接返回 ID 让 main 进入二级菜单
                return 1;
            } else {
                // 如果是科目，更新全局选择，并原地刷新屏幕显示 "selected"
                current_selected_task = selected_id;
                update_needed = 1;
                // 注意：这里不 return，让用户看到 selected 后可以继续按击掌启动
                // 或者你可以根据需求在此处返回任务 ID
                return selected_id;
            }
        }
        // 关键：在菜单界面也要检测“击掌”启动！
//        if(clap_sensor_detected() && current_selected_task > 1) {
//            return 100 + current_selected_task; // 特殊编码：代表带任务启动
//        }
        static uint8 trigger_locked = 0;
        if(!gpio_get_level(SWITCH1)) {
            if(trigger_locked == 0 && current_selected_task > 1) {
                trigger_locked = 1; // 上锁，本次触发有效
                return 100 + current_selected_task;
            }
        } else {
            trigger_locked = 0; // 开关拨回后解锁
        }
    }
}
/**
 * @brief 屏幕现实函数
 */
void screen_update(void){

    ips114_show_float(45, 16, pitch, 3, 1);
    ips114_show_float(45, 16*2, roll,  3, 1);
    ips114_show_float(45, 16*3, yaw,   3, 1);
    ips114_show_float(8*7, 16*5+5,speed, 3, 3);
    ips114_show_float(8*10, 16*6+5,distance, 5, 3);
    ips114_show_float(125+5+8*6, 16*1,  car_pose.x, 3, 2);
    ips114_show_float(125+5+8*6, 16*2, car_pose.y, 3, 2);
    ips114_show_float(125+8*7,16*4,gray_get_error(),2,1);
}
/**
 * @brief 渲染交互项（带颜色切换）
 * */
void menu2_render_actions(int current_flag) {
    for (int i = 0; i < RT_MENU_NUM; i++) {
        // 判断是否被选中
        if (i == (current_flag - 1)) {
            ips114_set_color(RGB565_WHITE, RGB565_BLUE);
        } else {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        }

        ips114_show_string(RealTimeMenuItems[i].x, RealTimeMenuItems[i].y,(const char *)RealTimeMenuItems[i].content);
    }
    // 恢复默认颜色，避免影响其他显示
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
}
/*
 * @ brief:二级菜单实时数据显示部分
 */
void menu2_RealTimeDisplay(void) {
    ips114_clear();
    int flag = 1;
    /* 绘制静态背景（只画一次） */
    ips114_show_string(5, 16,    "PIT:");
    ips114_show_string(5, 16*2,  "ROL:");
    ips114_show_string(5, 16*3,  "YAW:");
    ips114_show_string(5, 16*5+5,  "speed:");
    ips114_show_string(5,  16*6+5,  "distance:");
    ips114_show_string(125,  16*1, "POS X:");
    ips114_show_string(125,  16*2, "POS Y:");
    ips114_show_string(125,16*4,"G_ERR:");
    ips114_draw_line(0, 85, 239, 85, RGB565_GRAY);   // 横线
    ips114_draw_line(120, 0, 120, 85, RGB565_GRAY);  // 竖线

    while(true) {
        // 1. 渲染交互按键
        menu2_render_actions(flag);

        // 2. 数据更新与数值显示（你的原始 screen_update 逻辑）
        // 建议在这里直接写数值刷新的代码，或者确保 screen_update 只刷新数字区域
        screen_update();

        // 3. 按键逻辑
        if(GetKey_UP()) {
            if(--flag < 1) flag = RT_MENU_NUM;
            system_delay_ms(150);
        }
        if(GetKey_DOWN()) {
            if(++flag > RT_MENU_NUM) flag = 1;
            system_delay_ms(150);
        }
        if(GetKey_ENTER()) {
            system_delay_ms(200);
            if(flag == 1) { // 返回
                ips114_clear();
                return;
            }
            if(flag == 2) { // 执行清零动作
                if(RealTimeMenuItems[1].action != NULL) {
                    RealTimeMenuItems[1].action();
                }
            }
        }
    }
}

