#include "Ultrasonic.h"

// 低通滤波器构造函数
LowPassFilter::LowPassFilter(float timeConstant)
    : timeConstant(timeConstant), lastOutput(0.0f) {
    lastTimestamp = micros();
}

// 执行低通滤波
float LowPassFilter::filter(float input) {
    unsigned long currentTimestamp = micros();
    float dt = (currentTimestamp - lastTimestamp) * 1e-6f; // 时间间隔（秒）

    if (dt < 0.0f || dt > 0.5f) { // 如果时间过长或异常，重置为默认值
        dt = 1e-3f;
    }

    float alpha = timeConstant / (timeConstant + dt); // 计算平滑系数
    float output = alpha * lastOutput + (1.0f - alpha) * input;

    lastOutput = output;
    lastTimestamp = currentTimestamp;
    return output;
}

// 超声波测距构造函数
Ultrasonic::Ultrasonic(int trigPin, int echoPin)
    : trigPin(trigPin), echoPin(echoPin), filter(0.1f), lastDistance(0.0f) {}

// 初始化超声波模块引脚
void Ultrasonic::begin() {
    pinMode(trigPin, OUTPUT); // 设置触发引脚为输出
    pinMode(echoPin, INPUT);  // 设置回声引脚为输入
}

// 获取未滤波的测距值
float Ultrasonic::getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 10000); // 读取高电平持续时间，超时20ms   20 毫秒，对应最大测距 200 cm

    if (duration == 0) { // 如果测距超时，返回上次值
        return lastDistance;
    }

    float distance = duration * 0.034 / 2.0; // 将时间转换为距离（cm）
    if (distance > 100 || distance <= 3) {  // 异常值处理
        return lastDistance;
    }

    lastDistance = distance;
    return distance;
}

// 获取滤波后的测距值
float Ultrasonic::getFilteredDistance() {
    float rawDistance = getDistance();
    return filter.filter(rawDistance);
}
