// #include "RobotArm.h"

// // 初始化机械臂对象（舵机引脚为 3, 5, 6, 9, 10, 11）
// RobotArm arm(3, 5, 6, 9, 10, 11);

// // 目标坐标
// float target_x = 10.0;  // x 方向目标点
// float target_y = 12.0;  // y 方向目标点
// float target_z = 8.0;   // z 方向目标点

// void setup() {
//     Serial.begin(115200);  // 初始化串口，用于调试信息输出
//     arm.begin();           // 初始化机械臂

//     Serial.println("Starting Robot Arm...");

//     // 执行逆解算，计算目标舵机角度
//     if (arm.inverse_kinematics(target_x, target_y, target_z)) {
//         Serial.println("Inverse Kinematics Success!");

//         // 获取计算出的目标角度
//         int target_angles[6] = {arm.servo_angle[0], arm.servo_angle[1], arm.servo_angle[2], 
//                                 arm.servo_angle[3], arm.servo_angle[4], arm.servo_angle[5]};

//         // 执行轨迹控制，逐步移动到目标角度
//         arm.move_to_angles(target_angles);
//         Serial.println("Movement to Target Position Completed.");
//     } else {
//         Serial.println("Inverse Kinematics Failed: Target out of reach!");
//     }
// }

// void loop() {
//     // 空闲状态
// }
