/*
 * motion.c
 *
 *  Created on: 2026年5月8日
 *      Author: Wild_Mike_wzy
 */
#include "motion.h"
#include "zf_common_headfile.h"


MotionState_t current_state = IDLE;
float cmd_target_yaw = 0.0f;
float cmd_target_dist = 0.0f;
float start_dist = 0.0f; // 记录开始直行时的里程计数值
extern Pose_t car_pose;
PID_t line_track_pid;
//全局点位指针，数组索引号，数组大小
Point_t *current_path_ptr;
int path_index = 0;
int current_path_size;
/**
 * @brief 基础题（1）路径
 */
Point_t path1[] = {
    {1.0f, 0.0f}, // 点1
    {0.0f, 0.0f}  // 回到原点
};
int path1_size = sizeof(path1) / sizeof(Point_t);     //数组大小
/**
 * @brief 基础题（2）路径
 */
Point_t path2[] = {
        {0.0f,0.0f}
};
int path2_size = sizeof(path2) / sizeof(Point_t);     //数组大小
/**
 * @brief 触发任务
 * @param target_y 预期旋转角度
 * @param target_d 预期前进距离
 */
void run_motion_task(float target_y, float target_d) {
    cmd_target_yaw = target_y;
    cmd_target_dist = target_d;
    current_state = ROTATING; // 触发任务：先开始旋转
}
/**
 * @brief 切换/启动新任务
 */
void start_new_task(Point_t path_array[], int size) {
    current_path_ptr = path_array;
    current_path_size = size;
    path_index = 0;      // 关键：重置索引
    current_state = IDLE; // 确保进入空闲态触发逻辑
}
/**
 * @brief 自动取点逻辑
 */
void path_following_logic(void) {

    if (current_state == IDLE && path_index < current_path_size) {
        // 1. 获取当前目标点
        float target_x = current_path_ptr[path_index].x;
        float target_y = current_path_ptr[path_index].y;

        // 2. 计算偏差
        float dx = target_x - car_pose.x;
        float dy = target_y - car_pose.y;

        // 3. 计算需要行驶的距离
        cmd_target_dist = sqrtf(dx * dx + dy * dy);     //计算距离

        // 4. 计算绝对目标角度
        float angle_rad = atan2f(dy, dx);       //计算所需要的角度
        cmd_target_yaw = angle_rad * 180.0f / 3.14159265f;

        // 5. 角度归一化 (确保和你的 IMU 0~360 范围一致)
        if (cmd_target_yaw < 0) cmd_target_yaw += 360.0f;

        // 6. 激活任务
        current_state = ROTATING;
        run_motion_task(cmd_target_yaw,cmd_target_dist);
        // 7. 准备指向下一个点
        path_index++;
    }
}
/**
 * @brief 灰度循迹PID初始化
 */
void gray_track_PID_init(void){
    line_track_pid.err = 0.0;
    line_track_pid.last_err = 0.0;
    line_track_pid.kp = 0.005;
    line_track_pid.ki = 0.0;
    line_track_pid.kd = 0.15;
    line_track_pid.output = 0;
    line_track_pid.output_f = 0;
    line_track_pid.prev_err = 0;
    line_track_pid.target_val = 0;
}
/**
 * @brief 灰度循迹 PID 核心计算
 * @param pid: 指向你的 line_pid 结构体
 * @param current_error: 从 gray_get_error() 获取的偏差 (-40 ~ 40)
 * @return float: 返回转向速度修正值 (m/s)
 */
float gray_track_PID_realize(void) {
    // 1. 更新误差
    line_track_pid.err = gray_get_error();

    // 2. 比例项 (P)
    float p_out = line_track_pid.kp * line_track_pid.err;

    // 3. 微分项 (D)
    // 使用 (当前误差 - 上次误差) 来抑制震荡
    float d_out = line_track_pid.kd * (line_track_pid.err - line_track_pid.last_err);

    // 5. 计算总输出 (float 类型)
    line_track_pid.output_f = p_out + d_out;

    // 6. 更新历史误差记录
    line_track_pid.prev_err = line_track_pid.last_err; // 备份上上次误差 (对应你的结构体成员)
    line_track_pid.last_err = line_track_pid.err;      // 更新上次误差

    // 7. 返回计算结果
    return line_track_pid.output_f;
}

