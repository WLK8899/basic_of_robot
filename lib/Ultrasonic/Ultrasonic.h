#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
public:
    // 构造函数：初始化触发引脚和回声引脚
    Ultrasonic(int trigPin, int echoPin);

    // 初始化引脚
    void begin();

    // 测量距离，单位为厘米
    float getDistance();

private:
    int trigPin;   // 触发引脚
    int echoPin;   // 回声引脚
};

#endif
