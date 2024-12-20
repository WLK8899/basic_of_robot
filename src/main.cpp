#include <Arduino.h>

// #include "SoftSerialParser.h"
#include "HardSerialParser.h"
#include "Ultrasonic.h"
#include "LineFollower.h"
#include "RobotArm.h"
#include "MaxamWheel.h"
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
const int SERVO6_PIN = 8;

/*********************************************************************/

/***************************全局变量定义**************************/


int servoAngle[6] = {90, 0, 0, 0, 0, 0}; 
volatile float distance;

// 定义轮子的转速数组 (rad/s)
int wheelSpeeds[4] = {-200, -200, -200, -200};

// 创建 SerialParser 对象
// SoftSerialParser softParser(A1, A2);
HardSerialParser hardParser(Serial);
// 创建循迹对象
LineFollower lineFollower(leftSensorPin, rightSensorPin);
// 创建 Ultrasonic 对象
Ultrasonic ultrasonic(trigPin, echoPin);

RobotArm Arm(SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN);
/*********************************************************************/

/******************************定义函数************************************ */

void readSensors();

// 用户自定义命令解析回调函数
void commandHandler(char *tokens[], int tokenCount);

void rotation(int degree);
/*************************************************************************** */
// 目标坐标
float target_x = 0;  // x 方向目标点
float target_y = 246;  // y 方向目标点
float target_z = 100; // z 方向目标点
void setup()
{
  Serial.begin(115200);

  hardParser.setCommandCallback(commandHandler);
  // softParser.begin();
  // // 设置用户自定义命令回调函数
  // softParser.setCommandCallback(commandHandler);
  // 设置红外传感器引脚为输入
  lineFollower.begin();
  ultrasonic.begin(); // 初始化超声波引脚
  Arm.begin();        // 连接舵机到对应引脚

  // Arm.set_angle(servoAngle);
  Serial.println("Initialization completed");
}

void loop()
{

  // unsigned long startTime = micros();
  // readSensors();
  // lineFollower.followLine(leftOnLine, rightOnLine); // 调用循迹任务

  // unsigned long endTime = micros();
  // unsigned long executionTime = endTime - startTime;
  // Serial.print("Execution Time: ");
  // Serial.print(executionTime);
  // Serial.println(" microseconds");

  delay(1000);

      // //执行逆解算，计算目标舵机角度
      if (Arm.inverse_kinematics(target_x, target_y, target_z)) {
          Serial.println("Inverse Kinematics Success!");
      } else {
          Serial.println("Inverse Kinematics Failed: Target out of reach!");
      }
      delay(1000);
      if (Arm.inverse_kinematics(0, 180, 100)) {
          Serial.println("Inverse Kinematics Success!");
      } else {
          Serial.println("Inverse Kinematics Failed: Target out of reach!");
      }
  // delay(2000);

  //Arm.set_angle(servoAngle);
  //Arm.move_to_angles(servoAngle);
  // // 处理串口数据
  // //softParser.processSerial();
  hardParser.processSerial();
}

// ====== 读取传感器数据并处理 ====== //

void readSensors()
{
  // 使用digitalRead读取传感器状态
  int leftOnLine = digitalRead(leftSensorPin); // 黑线为低电平
  int rightOnLine = digitalRead(rightSensorPin);
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
    else if (target == 5)
    {
      servoAngle[5] = value;
      Serial.print("servoAngle[5] set to: ");
      Serial.println(servoAngle[5]);
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
      // Serial.print(leftOnLine);
      Serial.print(" | Right Sensor: ");
      // Serial.println(rightOnLine);

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

void sendCommand(int motor, int speed)
{
  char cmd_return_tmp[64];
  sprintf(cmd_return_tmp, "#%03dP%04dT0000!", motor, 1500 + speed);
  Serial.println(cmd_return_tmp);
}

void rotation(int degree)
{
  // 定义旋转速度
  const int ROTATION_SPEED = 465;
  const int TIME_PER_DEGREE = 8; // 单位延时时间(ms/degree)

  int rotationDirection = (degree > 0) ? ROTATION_SPEED : -ROTATION_SPEED; // 判断旋转方向

  // 设置电机速度
  sendCommand(6, rotationDirection);
  sendCommand(7, rotationDirection);
  sendCommand(8, rotationDirection);
  sendCommand(9, rotationDirection);

  // 计算旋转时间
  delay(TIME_PER_DEGREE * abs(degree));

  // 停止电机
  sendCommand(6, 0);
  sendCommand(7, 0);
  sendCommand(8, 0);
  sendCommand(9, 0);
}