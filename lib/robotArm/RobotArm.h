#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Arduino.h>
#include <Servo.h>

class RobotArm {
public:
    RobotArm(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6);
    void begin();
    void set_angle(int *angle); // 设置舵机角度
    bool inverse_kinematics(float target_x, float target_y, float target_z); // 逆解算，计算舵机角度并设置
    void move_to_angles(int *target_angles);  // 逐步移动到目标角度
    //int servo_angle[6]; // 保存舵机角度
    void set_calibration(int *offsets); // 设置舵机的校准偏移角度
    int servo_angle[6];   // 当前舵机角度 
private:
    int SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN;
    const float len_1 = 150;   // 底部圆盘高度  cm
    const float len_2 = 100;   // 机械臂长度 2
    const float len_3 = 100;   // 机械臂长度 3
    const float len_4 = 180;   // 机械臂长度 4
    const float bottom_r = 100;  // 底部圆盘半径

    
    int target_angle[6];
    Servo servo1, servo2, servo3, servo4, servo5, servo6;
    
    void calculate_angles(float target_x, float target_y, float target_z); // 核心逆解算函数

    // 检查目标点是否在机械臂可达范围内
    bool is_target_reachable(float target_x, float target_y, float target_z);

    void gradual_move(int servo_index, int target_angle);  // 单个舵机逐步移动
};

#endif
