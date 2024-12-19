#include "RobotArm.h"

RobotArm::RobotArm(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6)
{
    this->SERVO1_PIN = pin1;
    this->SERVO2_PIN = pin2;
    this->SERVO3_PIN = pin3;
    this->SERVO4_PIN = pin4;
    this->SERVO5_PIN = pin5;
    this->SERVO6_PIN = pin6;
}

void RobotArm::begin()
{
    // 连接舵机到对应引脚
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo5.attach(SERVO5_PIN);
    servo6.attach(SERVO6_PIN);
}

void RobotArm::set_angle(int *angle)
{
    servo1.write(angle[0]);
    servo2.write(angle[1]);
    servo3.write(angle[2]);
    servo4.write(angle[3]);
    servo5.write(angle[4]);
    servo6.write(angle[5]);
}