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
* 文件名称          isr
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

#include "isr_config.h"
#include "isr.h"
#include "zf_common_headfile.h"
// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。
extern float left_motor_speed;
extern float right_motor_speed;
extern float distance;
extern MotionState_t current_state;
extern float cmd_target_yaw;
extern float cmd_target_dist;
extern float start_dist; // 记录开始直行时的里程计数值
extern PID_t line_track_pid;
// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    //imu更新
    imu_update_attitude();
    //计算速度和里程
    calc_speed();
    calc_distance();
    //灰度传感器更新
    gray_update();
    //===================速度闭环测试=================================
    static int time = 0;
    time++;
//    if(time % 4 == 0){
//        int16 left_duty = speed_control_left_duty(0.3);
//        int16 right_duty = speed_control_right_duty(-0.3);
//        small_driver_set_duty(left_duty,right_duty);
//        printf("left speed:%f, right speed:%f\r\n", left_motor_speed, right_motor_speed);
//    }
        //===========================航向闭环测试====================================
//    if(time % 4 == 0){
//        uint8 status = !gpio_get_level(SWITCH1);
//        float target_yaw = 30.0f;
//        float turn_speed = direction_PID(target_yaw,yaw,gyro_z);
//        if(status){     //拨码开关开启，转弯90度
//            int16 left_duty = speed_control_left_duty(-turn_speed);
//            int16 right_duty = speed_control_right_duty(turn_speed);
//            small_driver_set_duty(left_duty,right_duty);
//            printf("left duty:%d, right duty:%d\r\n", left_duty, right_duty);
//        }
//    }
    //printf("T:%f, C:%f, E:%f, O:%f \r\n", target_yaw, yaw, dir.err, turn_speed);
    //==============航向环、速度环、距离环三环串级测试===================
//    if(time % 4 == 0){
//        update_position(yaw,distance);      //更新位置
//        // 2. 距离环计算 (得出基础速度)
//        // 假设我们要让小车精准行驶 3.0 米
//        float base_speed = distance_control(2.0f, distance);
//        printf("base_speed %f \r\n:",base_speed);
//        // 3. 航向环计算 (得出转向差速)
//        // 假设我们要让小车始终锁死在 90 度方向
//        float turn_speed = direction_PID(90.0f, yaw, gyro_z);
//
//        // 4. 融合输出
//        // 左轮目标 = 基础前进速度 - 转向修正
//        float left_target  = base_speed - turn_speed;
//        // 右轮目标 = 基础前进速度 + 转向修正
//        float right_target = base_speed + turn_speed;
//
//        // 5. 送入速度环执行
//        int16 left_duty  = speed_control_left_duty(left_target);
//        int16 right_duty = speed_control_right_duty(right_target);
//
//        small_driver_set_duty(left_duty, right_duty);
//    }
      //=========================坐标点闭环=============================
//      if(time % 4 == 0){        //降频，相当于20ms终端
//          float turn_speed = 0;     //转弯速度
//          float base_speed = 0;     //基础直线速度
//
//          switch (current_state) {
//              case IDLE:
//                  path_following_logic();
//                  break;
//
//              case ROTATING:
//                  // 只有航向环在工作
//                  turn_speed = direction_PID(cmd_target_yaw, yaw, gyro_z);
//                  base_speed = 0;
//
//                  // 判定旋转是否完成：误差小于 2 度 且 角速度足够小
//                  if (fabsf(cmd_target_yaw - yaw) < 2.0f && fabsf(gyro_z) < 5.0f) {
//                      start_dist = distance; // 记录当前里程计作为起点
//                      current_state = TRANSLATING;
//                  }
//                  break;
//
//              case TRANSLATING:
//                  // 航向环锁死角度，距离环开始输出速度
//                  turn_speed = direction_PID(cmd_target_yaw, yaw, gyro_z);
//
//                  // 相对距离闭环：当前行驶距离 = 总里程 - 记录的起点
//                  base_speed = distance_control(cmd_target_dist, distance - start_dist);
//
//                  // 判定直行是否完成：距离误差小于 3cm
//                  if (fabsf(cmd_target_dist - (distance - start_dist)) < 0.03f) {
//                      current_state = IDLE;
//                  }
//                  break;
//          }
//
//          // B. 融合并输出
//          small_driver_set_duty(speed_control_left_duty(base_speed - turn_speed),
//                               speed_control_right_duty(base_speed + turn_speed));
//
//          // C. 实时更新坐标（航位推算）
//          update_position(yaw, distance);
//      }
    //================循迹测试==================================
    if(time % 4 == 0) {
        float turn_speed = 0;
        float base_speed = 0.3f; // 循迹的基础速度

        //调用 PID 函数得到转向修正量
        turn_speed = gray_track_PID_realize();
        printf("turn_speed: %f \r\n",turn_speed);
        // 差速融合
        float left_target  = base_speed + turn_speed;
        float right_target = base_speed - turn_speed;

        // 输入到你之前的速度环
        small_driver_set_duty(speed_control_left_duty(left_target),
                             speed_control_right_duty(right_target));
        }
    // 别忘了更新坐标，即使在循迹，坐标系统也要运行
    update_position(yaw, distance);
}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);
    ras_uart_process();



    // 如果有检测目标，读取第0个并打印
    if (ras_get_detection_count() > 0)
    {
        ras_detection_t d;
        if (ras_get_detection(0, &d))
        {
            // 输出格式可按需修改
            printf("det:%u,%u,%u,%c\r\n", (unsigned)d.x, (unsigned)d.y, (unsigned)d.label_index, d.result_char);
        }
    }
}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);





}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);



    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);



    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
//IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // 开启中断嵌套
//
//#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
//        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
//#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
//}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //wireless_module_uart_handler();                 // 无线模块统一回调函数
    uart_control_callback();



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //gnss_uart_callback();                           // GNSS串口回调函数
    ras_uart_rx_callback();


}


IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** 串口中断函数 ****************************
