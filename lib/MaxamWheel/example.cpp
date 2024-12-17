#include <Arduino.h>
#include "MaxamWheel.h"

// 定义机器人的参数
const float WHEEL_BASE = 0.3;   // 前后轮距 0.3 米
const float WHEEL_TRACK = 0.3;  // 轮间距 0.3 米
const float WHEEL_RADIUS = 0.05; // 轮子半径 0.05 米

// 创建 MaxamWheel 对象
MaxamWheel mecanum(WHEEL_BASE, WHEEL_TRACK, WHEEL_RADIUS);

// 定义轮子的转速数组 (rad/s)
float wheelSpeeds[4];

// 机器人期望的速度
float desiredVx = 1.0;    // 前进 1 m/s
float desiredVy = 0.5;    // 侧向 0.5 m/s
float desiredOmega = 0.2; // 旋转 0.2 rad/s

void setup() {
    Serial.begin(9600);
    // 可选：设置最大轮速
    mecanum.setMaxWheelSpeed(50.0); // 根据电机规格设置
}

void loop() {
    // 计算各轮转速
    mecanum.inverseKinematics(desiredVx, desiredVy, desiredOmega, wheelSpeeds);

    // 输出轮速
    Serial.print("Wheel Speeds (rad/s): ");
    for(int i = 0; i < 4; i++) {
        Serial.print(wheelSpeeds[i]);
        if(i < 3) Serial.print(", ");
    }
    Serial.println();

    // 计算正向运动学验证
    float actualVx, actualVy, actualOmega;
    mecanum.forwardKinematics(wheelSpeeds, actualVx, actualVy, actualOmega);

    // 输出实际机器人速度
    Serial.print("Robot Velocity - Vx: ");
    Serial.print(actualVx);
    Serial.print(" m/s, Vy: ");
    Serial.print(actualVy);
    Serial.print(" m/s, Omega: ");
    Serial.print(actualOmega);
    Serial.println(" rad/s");

    delay(1000); // 每秒更新一次
}
