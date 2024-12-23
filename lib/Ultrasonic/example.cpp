#include <Arduino.h>
#include "Ultrasonic.h"

// 定义超声波模块引脚
#define TRIG_PIN 8
#define ECHO_PIN 9

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
    Serial.begin(9600); // 初始化串口通信
    ultrasonic.begin(); // 初始化超声波模块
}

void loop() {
    float rawDistance = ultrasonic.getDistance();          // 获取未滤波的距离
    float filteredDistance = ultrasonic.getFilteredDistance(); // 获取滤波后的距离

    Serial.print("Raw Distance: ");
    Serial.print(rawDistance);
    Serial.print(" cm, Filtered Distance: ");
    Serial.print(filteredDistance);
    Serial.println(" cm");

    delay(100); // 延迟100毫秒
}
