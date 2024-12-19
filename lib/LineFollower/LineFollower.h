#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include "MaxamWheel.h"

class LineFollower
{
public:
    // 构造函数：初始化传感器引脚和总线电机接口
    LineFollower(int leftSensor, int rightSensor);

    // 初始化
    void begin();

    // 循迹控制任务（放在loop中调用）
    void followLine(bool leftDetected, bool rightDetected);

private:
    // 传感器引脚
    int leftSensorPin;
    int rightSensorPin;

    MaxamWheel Wheel;
    // 速度参数
    const int MAX_SPEED = 600;       // 最大速度，范围-1000~1000
    const int TURN_SPEED = 400;      // 平移修正速度
    const int ROTATE_SPEED = 300;    // 旋转速度
    const int SMOOTH_INCREMENT = 10; // 平滑速度增量

    // 修正偏移方向机制  向右偏为正，向左偏为负
    int directionCounter;

    unsigned long offTrackStartTime;              // 记录脱线处理的开始时间
    const unsigned long MAX_OFFTRACK_TIME = 1000; // 最大脱线处理时间（毫秒）

    // 修正完成函数
    void isOK();
    void handleStraightLine();
    void handleOffTrack();
};

#endif
