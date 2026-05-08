#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"
// 宏定义
#define ACC_CONV_FACTOR     (9.79f / 4096.0f)
#define GYRO_CONV_FACTOR    (1.0f / 16.4f)

//#define ANGLE_BIAS          (1.8f)
#define EKF_Q_ANGLE         0.001f      // 过程噪声：角度预测的置信度 (越小越信陀螺仪)
#define EKF_Q_GYRO          0.003f      // 过程噪声：陀螺仪零偏的波动
#define EKF_R_ACCEL         0.5f        // 观测噪声：加速度计的置信度 (越小越信加速度计，越大越抗震)

#define ANGLE_BIAS         (1.0f)

#define CALIBRATION_SAMPLES 1000
#define VARIANCE_THRESHOLD  100.0f

// ==================== 数据结构 ====================
typedef struct {
    float q0, q1, q2, q3;   // 四元数 (代表姿态)
    float P[4][4];          // 状态协方差矩阵 (代表误差估计)
} EKF_State_t;


// 传感器数据
extern float acc_x, acc_y, acc_z;      // 加速度 (m/s²)
extern float gyro_x, gyro_y, gyro_z;   // 角速度 (°/s)

// 姿态角
extern float pitch, roll, yaw;         // 欧拉角 (度)

// 角速度
extern float gyro_pitch_rate, gyro_roll_rate, gyro_yaw_rate;  // (rad/s)

// 零偏和标准差
extern float gyro_bias_x, gyro_bias_y, gyro_bias_z;
extern float gyro_std_x, gyro_std_y, gyro_std_z;

// 状态标志
extern uint8_t is_calibrated;
extern uint8_t is_stable;

// 函数声明
void imu_init(void);
bool imu_calibrate(void);
void imu_update_attitude(void);
void ekf_reset(void);
#endif
