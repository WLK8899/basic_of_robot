#include <Arduino.h>
#include <TimerOne.h>
#include <Servo.h>

#include "MaxamWheel.h"
#include "SoftSerialParser.h"
#include "Ultrasonic.h"
/***************************引脚定义**************************/

// 红外传感器引脚
const int leftSensorPin = A5;  // 左红外传感器
const int rightSensorPin = A4; // 右红外传感器

// 定义超声波测距模块引脚
const int trigPin = A3; // 触发引脚连接 A3
const int echoPin = A0; // 回声引脚连接 A0

// 定义舵机信号线连接的引脚
const int SERVO1_PIN = 7;
const int SERVO2_PIN = 3;
const int SERVO3_PIN = 5;
const int SERVO4_PIN = 6;
const int SERVO5_PIN = 9;

/*********************************************************************/

/***************************全局变量定义**************************/
// 创建5个Servo对象
Servo servo1, servo2, servo3, servo4, servo5;
int servoAngle[5];

volatile bool timerFlag = false; // 定时器标志

// 红外传感器状态  在黑线上面变量为true
volatile bool leftOnLine = false;
volatile bool rightOnLine = false;
// volatile int leftOnLine = 0;
// volatile int rightOnLine = 0;

// 创建 Ultrasonic 对象
Ultrasonic ultrasonic(trigPin, echoPin);
volatile float distance;

// 定义地盘的参数
const float WHEEL_BASE = 0.3;    // 前后轮距 0.3 米
const float WHEEL_TRACK = 0.3;   // 轮间距 0.3 米
const float WHEEL_RADIUS = 0.05; // 轮子半径 0.05 米

// 创建 MaxamWheel 对象
MaxamWheel maxamwheel(WHEEL_BASE, WHEEL_TRACK, WHEEL_RADIUS);

// 定义轮子的转速数组 (rad/s)
int wheelSpeeds[4]={-200,-200,-200,-200};

float Vx = 1.0;    // 前进 1 m/s
float Vy = 0.5;    // 侧向 0.5 m/s
float Omega = 0.2; // 旋转 0.2 rad/s

//
char cmd_return_tmp[64];


// 创建 SerialParser 对象
SoftSerialParser softParser(A1, A2);

/*********************************************************************/

/******************************定义函数************************************ */

void timerISR();
void readSensors();
void followLine();

void set_speed(int *wheelSpeeds);
void set_angle(int *angle);

// 用户自定义命令解析回调函数
void commandHandler(char *tokens[], int tokenCount);
/*************************************************************************** */

void setup()
{

  Serial.begin(115200);
  softParser.begin();
  // 设置用户自定义命令回调函数
  softParser.setCommandCallback(commandHandler);
  // 设置红外传感器引脚为输入
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  ultrasonic.begin(); // 初始化超声波引脚

  // 连接舵机到对应引脚
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  // 初始化定时器
  Timer1.initialize(100000);        // 设置定时器每100ms触发一次
  Timer1.attachInterrupt(timerISR); // 关联中断服务程序

  Serial.println("Initialization completed");
}

void loop()
{
  // 定时器1任务
  if (timerFlag)
  {
    timerFlag = false;
     //readSensors();
    // followLine();
    //set_angle(servoAngle);
    set_speed(wheelSpeeds);
  }

  // 非定时任务

  // 处理串口数据
  softParser.processSerial();
}

void timerISR()
{
  timerFlag = true;
}

// ====== 读取传感器数据并处理 ====== //

void readSensors()
{
  // 使用digitalRead读取传感器状态
  leftOnLine = digitalRead(leftSensorPin) == LOW; // 黑线为低电平
  rightOnLine = digitalRead(rightSensorPin) == LOW;
  distance = ultrasonic.getDistance(); // 获取距离值

  // 调试输出
  Serial.print("Left Sensor: ");
  Serial.print(leftOnLine);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightOnLine);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void set_angle(int *angle)
{
  servo1.write(angle[0]);
  servo2.write(angle[1]);
  servo3.write(angle[2]);
  servo4.write(angle[3]);
  servo5.write(angle[4]);
}

void set_speed(int *wheelSpeeds)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 - wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);
  delay(20);                                                                // 解析ZMotor指令-左电机正向
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 - wheelSpeeds[0], 0); // 组合指令
  Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
  delay(20);
}

// 用户自定义命令解析回调函数
void commandHandler(char *tokens[], int tokenCount)
{
  // 检查命令格式是否正确
  if (tokenCount < 2)
  {
    Serial.println("Error: Invalid command format.");
    return;
  }

  // 第一个字段作为命令关键字
  char command = tokens[0][0];  // 取第一个字符
  int target = atoi(tokens[1]); // 第二个字段转为目标索引
  int value = atoi(tokens[2]);  // 第三个字段转为目标值

  // 使用 switch-case 处理命令
  switch (command)
  {
  case 'W': // 处理 "W" 命令：设置目标轮速变量值
    if (target == 0)
    {
      wheelSpeeds[0] = value;
      Serial.print("wheelSpeeds[0] set to: ");
      Serial.println(wheelSpeeds[0]);
    }
    else if (target == 1)
    {
      wheelSpeeds[1] = value;
      Serial.print("wheelSpeeds[1] set to: ");
      Serial.println(wheelSpeeds[1]);
    }
    else if (target == 2)
    {
      wheelSpeeds[2] = value;
      Serial.print("wheelSpeeds[2] set to: ");
      Serial.println(wheelSpeeds[2]);
    }

    else if (target == 3)
    {
      wheelSpeeds[3] = value;
      Serial.print("wheelSpeeds[3] set to: ");
      Serial.println(wheelSpeeds[3]);
    }
    else
    {
      Serial.println("Error: Invalid target index.");
    }
    break;

  case 'A': // 处理 "A" 命令：修改舵机值
    if (target == 0)
    {
      servoAngle[0] = value;
      Serial.print("servoAngle[0] value: ");
      Serial.println(servoAngle[0]);
    }
    else if (target == 1)
    {
      servoAngle[1] = value;
      Serial.print("servoAngle[1] set to: ");
      Serial.println(servoAngle[1]);
    }
    else if (target == 2)
    {
      servoAngle[2] = value;
      Serial.print("servoAngle[2] set to: ");
      Serial.println(servoAngle[2]);
    }

    else if (target == 3)
    {
      servoAngle[3] = value;
      Serial.print("servoAngle[3] set to: ");
      Serial.println(servoAngle[3]);
    }
    else if (target == 4)
    {
      servoAngle[4] = value;
      Serial.print("servoAngle[4] set to: ");
      Serial.println(servoAngle[4]);
    }
    else
    {
      Serial.println("Error: Invalid target index.");
    }
    break;

  case 'R': // 处理 "R" 命令：读取数组的变量值
    if (target == 1)
    {
      // 读取并输出 servoAngle 数组的值
      Serial.println("Servo Angles:");
      for (size_t i = 0; i < sizeof(servoAngle) / sizeof(servoAngle[0]); i++)
      {
        Serial.print("Servo ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(servoAngle[i]);
      }
    }
    else if (target == 2)
    {
      // 读取并输出 wheelSpeeds 数组的值
      Serial.println("Wheel Speeds:");
      for (size_t i = 0; i < sizeof(wheelSpeeds) / sizeof(wheelSpeeds[0]); i++)
      {
        Serial.print("Wheel ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(wheelSpeeds[i]);
      }
    }
    else if (target == 3)
    {
      Serial.print("Left Sensor: ");
      Serial.print(leftOnLine);
      Serial.print(" | Right Sensor: ");
      Serial.println(rightOnLine);

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    else
    {
      // 错误处理：target 无效
      Serial.println("Error: Invalid target index.");
    }
    break;

  default: // 未知命令
    Serial.print("Error: Unknown command '");
    Serial.print(command);
    Serial.println("'.");
    break;
  }
}
