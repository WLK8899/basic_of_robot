#include "LineFollower.h"

// 定义传感器引脚
const int LEFT_SENSOR = 2;   // 左红外传感器引脚
const int RIGHT_SENSOR = 3;  // 右红外传感器引脚
// 创建循迹对象
LineFollower lineFollower(LEFT_SENSOR, RIGHT_SENSOR);

void setup() {
    Serial.begin(115200);
    lineFollower.begin();
    Serial.println("Line Follower Initialized...");
}

void loop() {
    lineFollower.followLine(); // 调用循迹任务
    delay(10); // 控制任务频率
}