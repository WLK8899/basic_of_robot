#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include "MaxamWheel.h"
#include "SoftwareSerial.h"

class LineFollower
{
public:
    // 构造函数：传入串口对象
    LineFollower(Stream &serial);

    // 初始化
    void begin();

    // 循迹控制任务（放在 loop 中调用）
    void followLine();

private:
    // 串口
    Stream &serial;
    uint8_t buffer[14]; // 假设 buffer 长度为 18（以包含所有数据）
    int bufferIndex = 0;
    // 底盘控制类
    MaxamWheel Wheel;
    // 控制参数
    const int MAX_SPEED = 1000;    // 最大速度
    const int MAX_ROTATION = 1000; // 最大旋转速度
    // PID 控制参数
    const float Kp_linear = 17;   // 线性速度比例系数
    const float Ki_linear = 0.001;  // 线性速度积分系数
    const float Kd_linear = 0.06; // 线性速度微分系数

    const float Kp_angular = 50.49;    // 旋转速度比例系数
    const float Ki_angular = 0.00; // 旋转速度积分系数
    const float Kd_angular = 0.00; // 旋转速度微分系数

    // PID 控制变量
    float linear_error_prev = 0;  // 上次线性速度误差
    float angular_error_prev = 0; // 上次旋转速度误差
    float angular_error_sum = 0;  // 旋转速度积分误差

    float linear_error_sum = 0; // 线性速度积分误差

    unsigned long lastUpdateTime = 0;        // 上次更新时间
    const unsigned long updateInterval = 80; // 单位 ms（10 Hz）

    // 从 OpenMV 接收数据
    bool receiveOpenMVData(int32_t &VX, int32_t &VY, int32_t &OMEGA);
    // 计算速度
    void computeSpeed(int rho_err, int theta_err, int &vx, int &vy, int &omega);
};
#endif

// 根据openmv实际传来的数据来调参pi