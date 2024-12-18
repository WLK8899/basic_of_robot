#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
}

void Ultrasonic::begin() {
    pinMode(trigPin, OUTPUT); // 触发引脚为输出
    pinMode(echoPin, INPUT);  // 回声引脚为输入
}

float Ultrasonic::getDistance() {
    long duration;    // 高电平持续时间
    float distance;   // 计算得到的距离

    // 触发超声波模块发射信号
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 读取回声引脚的高电平持续时间
    duration = pulseIn(echoPin, HIGH);

    // 计算距离（单位：厘米）
    distance = duration * 0.034 / 2;

    return distance;
}
