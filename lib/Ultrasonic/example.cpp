#include "Ultrasonic.h"

// 定义超声波模块连接的引脚
const int trigPin = A3;  // 触发引脚
const int echoPin = A0;  // 回声引脚

// 创建 Ultrasonic 对象
Ultrasonic ultrasonic(trigPin, echoPin);

void setup() {
    Serial.begin(9600);   // 初始化串口
    ultrasonic.begin();   // 初始化超声波引脚
    Serial.println("Ultrasonic Distance Measurement");
}

void loop() {
    float distance = ultrasonic.getDistance(); // 获取距离值
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    delay(500); // 每500ms测量一次
}
