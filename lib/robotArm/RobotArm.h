#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Arduino.h>
#include <Servo.h>

class RobotArm {
public:
    RobotArm(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6);
    void begin();
    void set_angle(int *angle); // 设置舵机角度
    bool inverse_kinematics(float target_x, float target_y, float target_z,int i); // 逆解算，计算舵机角度并设置
    void move_to_angles(int *target_angles);  // 逐步移动到目标角度
    bool receiveOpenMVData(int &ball_color, int &px, int &py);
    void Coordinate_mapping(int &px, int &py);
    void Catch();
    void Release();
    int servo_angle[6];   // 当前舵机角度 
private:
    int SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN;
    const float len_1 = 160;   // 底部圆盘高度  mm
    const float len_2 = 100;   // 机械臂长度 2
    const float len_3 = 95;   // 机械臂长度 3
    const float len_4 = 165;   // 机械臂长度 4
    const float bottom_r = 65;  // 底部圆盘半径

    int bufferIndex = 0;
    uint8_t buffer[14];

    int target_angle[6];
    Servo servo1, servo2, servo3, servo4, servo5, servo6;
    
    void calculate_angles(float target_x, float target_y, float target_z); // 核心逆解算函数

    // 检查目标点是否在机械臂可达范围内
    
    bool is_target_reachable(float target_x, float target_y, float target_z);

    void gradual_move(int servo_index, int target_angle);  // 单个舵机逐步移动


   

    void catch_ball();
    void release_ball();
};

#endif
