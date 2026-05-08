/*
 * motion.c
 *
 *  Created on: 2026年5月8日
 *      Author: Wild_Mike_wzy
 */
#include "motion.h"
#include "zf_common_headfile.h"


MotionState_t current_state;
float cmd_target_yaw = 0.0f;
float cmd_target_dist = 0.0f;
float start_dist = 0.0f; // 记录开始直行时的里程计数值
extern Pose_t car_pose;
Point_t path[] = {
    {1.0f, 0.0f}, // 点1
    {1.0f, 1.0f}, // 点2
    {0.0f, 1.0f}, // 点3
    {0.0f, 0.0f}  // 回到原点
};
int path_index = 0;
int path_size = sizeof(path) / sizeof(Point_t);     //数组大小
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
 * @brief 自动取点逻辑
 */
void path_following_logic(void) {
    if (current_state == IDLE && path_index < path_size) {
        // 1. 获取当前目标点
        float target_x = path[path_index].x;
        float target_y = path[path_index].y;

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
        run_motion_task(cmd_target_yaw,cmd_target_dist);        //航向角70度，距离2m
        // 7. 准备指向下一个点
        path_index++;
    }
}


