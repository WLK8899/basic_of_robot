#ifndef MaxamWheel_H
#define MaxamWheel_H

#include <Arduino.h>


class MaxamWheel {
public:
    // 构造函数，初始化轮子位置和参数
    // wheelBase: 前后轮距
    // wheelTrack: 轮间距
    // wheelRadius: 轮子半径
    MaxamWheel(float wheelBase, float wheelTrack, float wheelRadius);

    // 逆向运动学：根据期望的机器人速度计算各个轮子的转速
    // vx: 前进速度 (m/s)
    // vy: 侧向速度 (m/s)
    // omega: 旋转速度 (rad/s)
    // Returns wheel speeds in radians per second
    void inverseKinematics(float vx, float vy, float omega, float* wheelSpeeds);

    // 正向运动学：根据各个轮子的转速计算机器人速度
    // wheelSpeeds: 各轮子的转速 (rad/s)
    // Returns vx, vy, omega by reference
    void forwardKinematics(float* wheelSpeeds, float &vx, float &vy, float &omega);

    // 设置轮子的最大转速（可选）
    void setMaxWheelSpeed(float maxSpeed);

    // 获取轮子的最大转速
    float getMaxWheelSpeed();

private:
    float L; // 前后轮距 (m)
    float W; // 轮间距 (m)
    float R; // 轮子半径 (m)
    float maxWheelSpeed; // 最大轮速 (rad/s)
};

#endif
