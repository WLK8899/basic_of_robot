#include "MaxamWheel.h"

MaxamWheel::MaxamWheel(float wheelBase, float wheelTrack, float wheelRadius) {
    L = wheelBase;
    W = wheelTrack;
    R = wheelRadius;
    maxWheelSpeed = 1000.0; // 默认最大转速，可以根据实际情况调整
}

void MaxamWheel::inverseKinematics(float vx, float vy, float omega, float* wheelSpeeds) {
    // 计算系数
    float a = vx - omega * (L + W) / 2;
    float b = vx + omega * (L + W) / 2;
    float c = vy - omega * (L + W) / 2;
    float d = vy + omega * (L + W) / 2;

    // 计算各轮转速    根据实际调整，最好直接输出··········
    // 前左轮 (Wheel 1)
    wheelSpeeds[0] = (b + d) / R;
    // 前右轮 (Wheel 2)
    wheelSpeeds[1] = (b - d) / R;
    // 后左轮 (Wheel 3)
    wheelSpeeds[2] = (a + c) / R;
    // 后右轮 (Wheel 4)
    wheelSpeeds[3] = (a - c) / R;

    // 找到绝对值最大的转速
    float maxSpeed = abs(wheelSpeeds[0]);
    for(int i = 1; i < 4; i++) {
        if(abs(wheelSpeeds[i]) > maxSpeed) {
            maxSpeed = abs(wheelSpeeds[i]);
        }
    }

    // 如果最大转速超过限制，按比例缩放
    if(maxSpeed > maxWheelSpeed) {
        float scale = maxWheelSpeed / maxSpeed;
        for(int i = 0; i < 4; i++) {
            wheelSpeeds[i] *= scale;
        }
    }
}

void MaxamWheel::forwardKinematics(float* wheelSpeeds, float &vx, float &vy, float &omega) {
    // 计算机器人速度的逆矩阵
    // 使用简单的平均来估计
    float a = R / 4.0;
    vx = a * (wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] + wheelSpeeds[3]);
    vy = a * (-wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] - wheelSpeeds[3]);
    omega = a * (-wheelSpeeds[0] + wheelSpeeds[1] - wheelSpeeds[2] + wheelSpeeds[3]) / (L + W);
}

void MaxamWheel::setMaxWheelSpeed(float maxSpeed) {
    maxWheelSpeed = maxSpeed;
}

float MaxamWheel::getMaxWheelSpeed() {
    return maxWheelSpeed;
}
