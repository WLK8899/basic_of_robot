#include <Arduino.h>
#define M1 7
#define M2 3
#define M3 5
#define M4 6
#define M5 9
#define M6 8

// 数字引脚a5 ，a4
char cmd_return_tmp[64];
int left_led, right_led;
void set_speed(int Va, int Vb, int Vc, int Vd)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + Va, 200); // 组合指令
  Serial.println(cmd_return_tmp);                               // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 - Vb, 200); // 组合指令
  Serial.println(cmd_return_tmp);                               // 解析ZMotor指令-左电机正向
  delay(20);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + Vc, 200); // 组合指令
  Serial.println(cmd_return_tmp); 
  delay(20);                              // 解析ZMotor指令-左电机正向
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 - Vd, 200); // 组合指令
  Serial.println(cmd_return_tmp);                               // 解析ZMotor指令-左电机正向
  delay(20);
}

// 控制麦克萨姆轮小车前进
#define FLOW 250
void moveForward()
{
  set_speed(FLOW, FLOW, FLOW, FLOW);
}

// 控制麦克萨姆轮小车后退
#define BACK -200
void moveBackward()
{
  set_speed(BACK, BACK, BACK, BACK);
}

// 向左转
#define LEFT 250
void turnLeft()
{
  set_speed(0, LEFT, LEFT, 0);
}

// 向右转
#define RIGHT 250
void turnRight()
{
  set_speed(RIGHT, 0, 0, RIGHT);
}

// 控制循迹
void lineFollowing()
{
  int leftSensor = digitalRead(A5);
  int rightSensor = digitalRead(A4);
  Serial.println(leftSensor);
  Serial.println(rightSensor);

  if (leftSensor == HIGH && rightSensor == HIGH)
  {
    // 如果前方有线，前进
    moveForward();
  }
  else if (leftSensor == HIGH && rightSensor == LOW)
  {
    // 如果左边有线，向右转
    turnRight();
  }
  else if (rightSensor == HIGH && leftSensor == LOW)
  {
    // 如果右边有线，向左转
    turnLeft();
  }
  else if (rightSensor == LOW && leftSensor == LOW)
  {
    // 如果后方有线，后退
    moveBackward();
  }
  else
  {
  }
}
void setup()
{
  Serial.begin(115200);
  // pinMode(M1, OUTPUT);
  // pinMode(M2, OUTPUT);
  // pinMode(M3, OUTPUT);
  // pinMode(M4, OUTPUT);
  // pinMode(M5, OUTPUT);
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
}

void loop()
{

delay(50);
  lineFollowing();
}
