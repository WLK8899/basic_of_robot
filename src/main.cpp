#include <Arduino.h>
#include <TimerOne.h>
#include <Servo.h>

#include "MaxamWheel.h"

/***************************引脚定义**************************/

// 红外传感器引脚
const int leftSensorPin = A5;  // 左红外传感器
const int rightSensorPin = A4; // 右红外传感器

// 定义舵机信号线连接的引脚
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 5;
const int SERVO3_PIN = 6;
const int SERVO4_PIN = 9;
const int SERVO5_PIN = 10;

/*********************************************************************/

/***************************全局变量定义**************************/
// 创建5个Servo对象
Servo servo1, servo2, servo3, servo4, servo5;
int servoAngle[5];


volatile bool timerFlag = false; // 定时器标志

// 红外传感器状态
volatile bool leftOnLine = false;
volatile bool rightOnLine = false;

// 定义地盘的参数
const float WHEEL_BASE = 0.3;    // 前后轮距 0.3 米
const float WHEEL_TRACK = 0.3;   // 轮间距 0.3 米
const float WHEEL_RADIUS = 0.05; // 轮子半径 0.05 米

// 创建 MaxamWheel 对象
MaxamWheel maxamwheel(WHEEL_BASE, WHEEL_TRACK, WHEEL_RADIUS);

// 定义轮子的转速数组 (rad/s)
float wheelSpeeds[4];

float Vx = 1.0;    // 前进 1 m/s
float Vy = 0.5;    // 侧向 0.5 m/s
float Omega = 0.2; // 旋转 0.2 rad/s

// 
char cmd_return_tmp[64];

/*********************************************************************/

/******************************定义函数************************************ */

void timerISR();
void readSensors();
void followLine();

void set_speed(float *wheelSpeeds);
void set_angle(int *angle);
/*************************************************************************** */

void setup()
{

  Serial.begin(115200);
  // 设置红外传感器引脚为输入

    // 连接舵机到对应引脚
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // 初始化定时器
  Timer1.initialize(100000); // 设置定时器每100ms触发一次
  Timer1.attachInterrupt(timerISR); // 关联中断服务程序

  Serial.println("Initialization completed");
}

void loop()
{
  if (timerFlag) {
    timerFlag = false;
    readSensors();
    followLine();
  }
}

void timerISR() {
  timerFlag = true;
}

// ====== 读取传感器数据并处理 ====== //

void readSensors() {
  // 使用digitalRead读取传感器状态
  leftOnLine = digitalRead(leftSensorPin) == LOW;  // 假设黑线为低电平
  rightOnLine = digitalRead(rightSensorPin) == LOW;

  // 调试输出
  Serial.print("Left Sensor: ");
  Serial.print(leftOnLine ? "ON" : "OFF");
  Serial.print(" | Right Sensor: ");
  Serial.println(rightOnLine ? "ON" : "OFF");
}

void set_angle(int *angle){

    servo1.write(angle[0]);
    servo2.write(angle[1]);
    servo3.write(angle[2]);
    servo4.write(angle[3]);
    servo5.write(angle[4]);

}

void set_speed(float *wheelSpeeds)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);
  delay(20);                                                                // 解析ZMotor指令-左电机正向
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
}