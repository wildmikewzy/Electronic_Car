#include "zf_common_headfile.h"

#include <math.h>
#include <string.h> // for memset

// 引入你的底层驱动头文件 (请根据实际情况修改)

//float ANGLE_BIAS=1.8f;
#define DT (PIT_t/1000.0f)
// ==================== 全局变量定义 ====================
// 传感器物理量数据
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;

// 姿态角 (欧拉角)
float pitch = 0, roll = 0, yaw = 0;

// 陀螺仪零偏
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
float gyro_std_x = 0, gyro_std_y = 0, gyro_std_z = 0;

// 状态标志
uint8_t is_calibrated = 0;
uint8_t is_stable = 0;

// EKF 核心结构体实例
static EKF_State_t ekf;

// ==================== 内部辅助函数 ====================

// 快速平方根倒数 (经典的 Quake III 算法，比 1.0/sqrtf 快)
static float inv_sqrt(float x) {
    if (x <= 0) return 0;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// EKF 初始化
 void ekf_reset(void) {
    // 1. 立即读取一次当前加速度值
    imu660rx_get_acc();

    // 2. 坐标系修正 (RFU: X右, Y前, Z上)
    // 根据之前的结论：Z轴读数向下，需取反
    float ax = -(float)imu660rx_acc_x;
    float ay =  (float)imu660rx_acc_y;
    float az = -(float)imu660rx_acc_z;

    // 3. 计算初始欧拉角 (弧度)
    // 防止除以0
    float init_pitch = 0.0f;
    float init_roll  = 0.0f;


        // Pitch (绕X轴/右轴转): 车头抬起时，Y轴分量变化
        // 使用 -ay 是为了符合右手定则
        init_pitch = -atan2f(-ay, sqrtf(ax*ax + az*az));

        // Roll (绕Y轴/前轴转): 车身侧倾时，X轴分量变化
        init_roll  = -atan2f(ax, az);


    // 4. 将初始欧拉角转换为四元数 (Z-Y-X 顺序)
    // 既然要简洁，这里直接展开计算，不调用额外函数
    float cp = cosf(init_pitch * 0.5f);
    float sp = sinf(init_pitch * 0.5f);
    float cr = cosf(init_roll * 0.5f);
    float sr = sinf(init_roll * 0.5f);
    // 初始 Yaw = 0
    float cy = 1.0f;
    float sy = 0.0f;

    ekf.q0 = cy * cr * cp + sy * sr * sp;
    ekf.q1 = cy * cr * sp - sy * sr * cp; // Pitch部分
    ekf.q2 = cy * sr * cp + sy * cr * sp; // Roll部分
    ekf.q3 = sy * cr * cp - cy * sr * sp;

    // 5. 初始化协方差矩阵 P
    memset(ekf.P, 0, sizeof(ekf.P));
    ekf.P[0][0] = 0.005f;
    ekf.P[1][1] = 0.005f;
    ekf.P[2][2] = 0.005f;
    ekf.P[3][3] = 0.005f;

    // 6. 立即更新全局变量 (让上层应用能直接读到非0值)
    pitch = init_pitch * RAD_TO_DEG;
    roll  = init_roll  * RAD_TO_DEG;
    yaw   = 0.0f;
}

// ==================== 外部接口函数 ====================

// IMU初始化
void imu_init(void)
{
    // 底层硬件初始化
    while(imu660rx_init()) {
        gpio_toggle_level(P20_9);
        system_delay_ms(100);
    }

    // 复位数据
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;

    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;

    is_calibrated = 0;
    is_stable = 0;
    // 初始化 EKF 状态
    //ekf_reset();

}

// IMU校准 (保持你原有的逻辑，稍作整理)
bool imu_calibrate(void)
{
    int i;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0; // 使用 double 防止溢出
    double sum_sq_gx = 0, sum_sq_gy = 0, sum_sq_gz = 0;
    float gx, gy, gz;

    // 采集样本
    for(i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        imu660rx_get_gyro();

        gx = -(float)imu660rx_gyro_x * GYRO_CONV_FACTOR;
        gy =  (float)imu660rx_gyro_y * GYRO_CONV_FACTOR;
        gz = -(float)imu660rx_gyro_z * GYRO_CONV_FACTOR;

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        sum_sq_gx += gx * gx;
        sum_sq_gy += gy * gy;
        sum_sq_gz += gz * gz;

         system_delay_ms(5);
    }

    // 计算均值
    gyro_bias_x = (float)(sum_gx / CALIBRATION_SAMPLES);
    gyro_bias_y = (float)(sum_gy / CALIBRATION_SAMPLES);
    gyro_bias_z = (float)(sum_gz / CALIBRATION_SAMPLES);

    // 计算方差
    float var_x = (float)(sum_sq_gx / CALIBRATION_SAMPLES) - (gyro_bias_x * gyro_bias_x);
    float var_y = (float)(sum_sq_gy / CALIBRATION_SAMPLES) - (gyro_bias_y * gyro_bias_y);
    float var_z = (float)(sum_sq_gz / CALIBRATION_SAMPLES) - (gyro_bias_z * gyro_bias_z);

    if(var_x < 0) var_x = 0;
    if(var_y < 0) var_y = 0;
    if(var_z < 0) var_z = 0;

    gyro_std_x = sqrtf(var_x);
    gyro_std_y = sqrtf(var_y);
    gyro_std_z = sqrtf(var_z);

    // 判断静止
    is_stable = (var_x < VARIANCE_THRESHOLD) &&
                (var_y < VARIANCE_THRESHOLD) &&
                (var_z < VARIANCE_THRESHOLD);

    if(is_stable) {
        is_calibrated = 1;
        // 校准后重置 EKF 以消除校准期间的漂移
        ekf_reset();
    }

    return is_stable;
}

// 核心姿态解算函数 (EKF 实现)
void imu_update_attitude(void)
{
    float norm;
    float half_dt = 0.5f * DT;

    // 1. 读取原始数据
    imu660rx_get_acc();
    imu660rx_get_gyro();

    // 2. 坐标系变换 (根据你的硬件: X右, Y前, Z上)
    // 之前分析结论：Acc Z 实际向下需取反，Gyro Z 需同步取反
    float ax = -(float)imu660rx_acc_x * ACC_CONV_FACTOR;
    float ay =  (float)imu660rx_acc_y * ACC_CONV_FACTOR;
    float az = -(float)imu660rx_acc_z * ACC_CONV_FACTOR;

    float gx_rad = (-(float)imu660rx_gyro_x * GYRO_CONV_FACTOR - gyro_bias_x) * DEG_TO_RAD;
    float gy_rad = ( (float)imu660rx_gyro_y * GYRO_CONV_FACTOR - gyro_bias_y) * DEG_TO_RAD;
    float gz_rad = (-(float)imu660rx_gyro_z * GYRO_CONV_FACTOR - gyro_bias_z) * DEG_TO_RAD;

    // 更新调试变量
    acc_x = ax; acc_y = ay; acc_z = az;
    gyro_x = gx_rad * RAD_TO_DEG; gyro_y = gy_rad * RAD_TO_DEG; gyro_z = gz_rad * RAD_TO_DEG;

    // ==================== EKF 步骤 1: 预测 ====================
    float q0 = ekf.q0;
    float q1 = ekf.q1;
    float q2 = ekf.q2;
    float q3 = ekf.q3;

    ekf.q0 += (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * half_dt;
    ekf.q1 += ( q0 * gx_rad - q3 * gy_rad + q2 * gz_rad) * half_dt;
    ekf.q2 += ( q3 * gx_rad + q0 * gy_rad - q1 * gz_rad) * half_dt;
    ekf.q3 += (-q2 * gx_rad + q1 * gy_rad + q0 * gz_rad) * half_dt;

    norm = inv_sqrt(ekf.q0 * ekf.q0 + ekf.q1 * ekf.q1 + ekf.q2 * ekf.q2 + ekf.q3 * ekf.q3);
    ekf.q0 *= norm; ekf.q1 *= norm; ekf.q2 *= norm; ekf.q3 *= norm;

    ekf.P[0][0] += EKF_Q_ANGLE * DT;
    ekf.P[1][1] += EKF_Q_ANGLE * DT;
    ekf.P[2][2] += EKF_Q_ANGLE * DT;
    ekf.P[3][3] += EKF_Q_ANGLE * DT;

    // ==================== EKF 步骤 2: 更新 ====================
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= norm; ay *= norm; az *= norm;

        float v_x = 2 * (ekf.q1 * ekf.q3 - ekf.q0 * ekf.q2);
        float v_y = 2 * (ekf.q0 * ekf.q1 + ekf.q2 * ekf.q3);
        float v_z = ekf.q0 * ekf.q0 - ekf.q1 * ekf.q1 - ekf.q2 * ekf.q2 + ekf.q3 * ekf.q3;

        float err_x = ay * v_z - az * v_y;
        float err_y = az * v_x - ax * v_z;
        float err_z = ax * v_y - ay * v_x;
       // float err_z = 0;
        float gain = 0.5f * DT;

        ekf.q0 += (-ekf.q1 * err_x - ekf.q2 * err_y - ekf.q3 * err_z) * gain;
        ekf.q1 += ( ekf.q0 * err_x - ekf.q3 * err_y + ekf.q2 * err_z) * gain;
        ekf.q2 += ( ekf.q3 * err_x + ekf.q0 * err_y - ekf.q1 * err_z) * gain;
        ekf.q3 += (-ekf.q2 * err_x + ekf.q1 * err_y + ekf.q0 * err_z) * gain;

        norm = inv_sqrt(ekf.q0 * ekf.q0 + ekf.q1 * ekf.q1 + ekf.q2 * ekf.q2 + ekf.q3 * ekf.q3);
        ekf.q0 *= norm; ekf.q1 *= norm; ekf.q2 *= norm; ekf.q3 *= norm;
    }

    // ==================== 结果输出: 修正后的欧拉角 ====================
    // 你的坐标系：X=右，Y=前，Z=上

    // 1. Roll (绕 Y 轴旋转): 横滚
    // 使用 asin 计算中间轴的旋转，范围限制在 -90 到 90 度
    // 对应标准公式中的 "Pitch" 位置，但在你的坐标系里它是 Roll
    float sin_pitch = 2 * (ekf.q0 * ekf.q2 - ekf.q3 * ekf.q1); // q2对应Y轴分量
    if (fabs(sin_pitch) >= 1)
        pitch = copysign(90.0f, sin_pitch);
    else
        pitch = asin(sin_pitch) * RAD_TO_DEG;

    // 2. Pitch (绕 X 轴旋转): 俯仰
    // 使用 atan2 计算，范围 -180 到 180 度
    // 对应标准公式中的 "Roll" 位置，但在你的坐标系里它是 Pitch



    roll = atan2(2 * (ekf.q0 * ekf.q1 + ekf.q2 * ekf.q3), 1 - 2 * (ekf.q1 * ekf.q1 + ekf.q2 * ekf.q2)) * RAD_TO_DEG+ANGLE_BIAS;

    // 3. Yaw (绕 Z 轴旋转): 航向
    yaw = atan2(2 * (ekf.q0 * ekf.q3 + ekf.q1 * ekf.q2), 1 - 2 * (ekf.q2 * ekf.q2 + ekf.q3 * ekf.q3)) * RAD_TO_DEG;

    if (yaw < 0.0f) yaw += 360.0f;
}

