#include "LineFollower.h"

// 构造函数
LineFollower::LineFollower(int leftSensor, int rightSensor)
    : leftSensorPin(leftSensor), rightSensorPin(rightSensor)
{
    directionCounter = 0;
}

// 初始化传感器
void LineFollower::begin()
{
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
}

// 循迹主任务
void LineFollower::followLine()
{
    bool leftDetected = digitalRead(leftSensorPin);
    bool rightDetected = digitalRead(rightSensorPin);

    if (leftDetected && rightDetected)
    {
        isOK();
        // 正常直线行驶
        offTrackStartTime = 0; // 重置脱线时间记录
        handleStraightLine();
    }
    else if (leftDetected && !rightDetected)
    {
        isOK();
        // 向右偏移，左平移修正
        if (directionCounter <= 20)
        {
            directionCounter++;
        }
        else
            directionCounter = 20;

        offTrackStartTime = 0; // 重置脱线时间记录

        handleStraightLine();
    }
    else if (!leftDetected && rightDetected)
    {
        isOK();
        // 向左偏移，右平移修正
        if (directionCounter >= -20)
        {
            directionCounter--;
        }
        else
            directionCounter = -20;
        offTrackStartTime = 0; // 重置脱线时间记录

        handleStraightLine();
    }
    else
    {
        // 两个传感器都未检测到黑线，脱线处理
        handleOffTrack();
    }
}

// // 直线加速
void LineFollower::handleStraightLine()
{
    Wheel.set_speed(MAX_SPEED, 0, 0);
}

// 脱线处理逻辑
void LineFollower::handleOffTrack()
{
    // 获取当前时间
    unsigned long currentTime = millis();

    // 如果进入脱线状态的时间未记录，记录当前时间
    if (offTrackStartTime == 0)
    {
        offTrackStartTime = currentTime;
    }

    // 检查是否超时
    if (currentTime - offTrackStartTime > MAX_OFFTRACK_TIME)
    {
        if (directionCounter > 0)
        {
            Wheel.set_speed(300, -TURN_SPEED / 2, 0);
            // delay(100);
        }
        else if (directionCounter < 0)
        {
            Wheel.set_speed(300, TURN_SPEED / 2, 0);
            // delay(100);
        }
        else
        {
            Wheel.set_speed(0, 0, ROTATE_SPEED);
            // delay(100);
        }

        // 超过最大时间，切换到原地旋转模式
        offTrackStartTime = 0; // 重置时间记录
        return;
    }
}

void LineFollower::isOK()
{
    if (offTrackStartTime > 0)
    {
        directionCounter = 0;
        offTrackStartTime = 0;
    }
}
