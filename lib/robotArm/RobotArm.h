#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "Servo.h"
class RobotArm{

public:
    RobotArm (int pin1,int pin2,int pin3 ,int pin4,int pin5 ,int pin6);
    void begin();
    void set_angle(int *angle);

private:
    int SERVO1_PIN,SERVO2_PIN,SERVO3_PIN,SERVO4_PIN,SERVO5_PIN,SERVO6_PIN;

    Servo servo1, servo2, servo3, servo4, servo5 , servo6 ;
    int servo_angle[6];
      
    

};

#endif