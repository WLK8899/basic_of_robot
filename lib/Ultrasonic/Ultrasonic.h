#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// 低通滤波器类
class LowPassFilter {
public:
    LowPassFilter(float timeConstant); // 构造函数，初始化时间常数
    ~LowPassFilter() = default;       // 默认析构函数
    float filter(float input);        // 执行滤波操作
private:
    float timeConstant;               // 滤波器时间常数
    unsigned long lastTimestamp;      // 上一次执行的时间戳
    float lastOutput;                 // 上一次的输出值
};

// 超声波测距类
class Ultrasonic {
public:
    Ultrasonic(int trigPin, int echoPin); // 构造函数，初始化触发和回声引脚
    void begin();                         // 初始化引脚
    float getDistance();                  // 测量距离（单位：cm）
    float getFilteredDistance();          // 获取滤波后的距离
    void corrDistance();

private:
    int trigPin;                          // 触发引脚
    int echoPin;                          // 回声引脚
    LowPassFilter filter;                 // 低通滤波器实例
    float lastDistance;                   // 上一次的距离值

};

#endif // ULTRASONIC_H
