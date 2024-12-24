#include <Arduino.h>
#include "LineFollower.h"

// 使用软件串口（如果硬件串口占用，切换为 SoftwareSerial）
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX = 10, TX = 11

LineFollower follower(mySerial);

void setup()
{
    Serial.begin(115200); // 调试信息
    mySerial.begin(115200); // 用于接收 OpenMV 数据

    follower.begin(); // 初始化
}

void loop()
{
    follower.followLine(); // 执行循迹任务
}
