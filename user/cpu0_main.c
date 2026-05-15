/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

//=============宏定义/全局变量==================
void init_all(void);

#define Sound                   (ERU_CH3_REQ6_P02_0)
//extern volatile uint8 clap_flag;

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    init_all();     //初始化函数
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************
void init_all(void){
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);        //蜂鸣器初始化
    gpio_init(LED1,GPO,GPIO_LOW,GPO_PUSH_PULL);     //LED灯（红）初始化

    motor_init();       //无刷电机初始化
    menu_init();    //菜单初始化
    gray_init();        //灰度传感器初始化
    Magnet_init();      //电磁铁初始化
    Key_init();
    // 初始化IMU
    printf("Initializing IMU...\r\n");
    ips114_show_string(0,0,"loading");
    imu_init();         //imu初始化，校准零偏
    // 校准IMU
    printf("Calibrating...\r\n");
    while(!imu_calibrate());
    printf("Calibration done\r\n");
    printf("IMU Initializing Done");
    ips114_clear();
    ras_uart_init();        //串口初始化
    pit_ms_init(CCU60_CH0,PIT_t);       //CH0定时中断初始化
    pit_ms_init(CCU60_CH1,10);          //CH1定时中断初始化（树莓派）
    speed_control_init();       //速度环控制初始化
    direction_PID_init();       //航向换参数初始化
    distance_PID_init();        //距离环参数初始化
    gray_track_PID_init();      //灰度循迹参数初始化
    image_control_init();       // 视觉闭环参数初始化
    servo_init();       //舵机初始化
    exti_init(Sound, EXTI_TRIGGER_FALLING);
}
