#include "RobotArm.h"

// 构造函数
RobotArm::RobotArm(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6)
{
    this->SERVO1_PIN = pin1;
    this->SERVO2_PIN = pin2;
    this->SERVO3_PIN = pin3;
    this->SERVO4_PIN = pin4;
    this->SERVO5_PIN = pin5;
    this->SERVO6_PIN = pin6;
}
// // 设置舵机的校准偏移角度
// void RobotArm::set_calibration(int *offsets) {
//     for (int i = 0; i < 6; i++) {
//         calibration[i] = offsets[i];
//     }
// }
// 初始化舵机
void RobotArm::begin()
{
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo5.attach(SERVO5_PIN);
    servo6.attach(SERVO6_PIN);


}

// 设置舵机角度
void RobotArm::set_angle(int *angle)
{

    servo1.write(static_cast<int>((30+angle[0] *0.7778) ));
    delay(50);
    servo2.write(static_cast<int>((55+angle[1] *0.7778) ));
    delay(50);
    servo3.write(static_cast<int>((20+(angle[2]+90) *0.7778) ));
    delay(50);
    servo4.write(static_cast<int>((25+(angle[3]+90) *0.7778) ));
    delay(50);
    servo5.write(static_cast<int>((angle[4] *0.7778) ));
    delay(50);
    servo6.write(static_cast<int>((angle[5] *0.7778) ));
}

// 检查目标点是否在可达范围内
bool RobotArm::is_target_reachable(float target_x, float target_y, float target_z)
{

    float maxReach = len_2 + len_3 + len_4;
    float minReach = abs(len_2 - len_3);
    float targetLen = sqrt(target_x * target_x + target_y * target_y);

    // 检查水平距离是否在范围内
    if (targetLen < minReach || targetLen > maxReach)
    {
        return false;
    }

    // 检查高度是否在范围内
    if (target_z > (len_1 + len_4) || target_z < 0)
    {
        return false;
    }

    return true;
}

// 逆解算：根据目标坐标计算舵机角度
bool RobotArm::inverse_kinematics(float target_x, float target_y, float target_z)
{
    if (!is_target_reachable(target_x, target_y, target_z))
    {
        Serial.println("Target out of reach!");
        return false;
    }

    calculate_angles(target_x, target_y, target_z); // 调用核心逆解算函数
    // 验证计算出的舵机角度是否合法
    if (target_angle[1] >= -30 && target_angle[2] >= 0 && target_angle[3] >= 0 &&
        target_angle[1] <= 90 && target_angle[2] <= 180 && target_angle[3] <= 180)
    {
        //move_to_angles(target_angle); // 如果角度合法，设置舵机角度
        set_angle(target_angle);
        return true;
    }

    Serial.println("Invalid joint angles!");
    return false;
}

// 核心逆解算函数
void RobotArm::calculate_angles(float target_x, float target_y, float target_z)
{
    float j1, j2, j3, j4;
    float L, H, j_sum;
    float len, high;
    float cos_j3, sin_j3;
    float cos_j2, sin_j2;
    float k1, k2;
    int i;
    float n = 0, m = 0;

    // // 限制 target_y 范围
    // if (target_y >= 18)
    //     target_y = 18;
    // else if (target_y <= 3)
    //     target_y = 3;

    // 计算基座旋转角度 j1
    if (target_x == 0)
        j1 = 90;
    else
        j1 = 90 - atan(target_x / (target_y + bottom_r)) * 57.3;

    // 遍历寻找解
    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180;
        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
        high = target_z;

        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;

        cos_j3 = ((L * L) + (H * H) - (len_2 * len_2) - (len_3 * len_3)) / (2 * len_2 * len_3);
        if (cos_j3 < -1 || cos_j3 > 1)
            continue; // 跳过非法值

        sin_j3 = sqrt(1 - cos_j3 * cos_j3);
        j3 = atan2(sin_j3, cos_j3) * 57.3;

        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        if (cos_j2 < -1 || cos_j2 > 1)
            continue; // 跳过非法值

        sin_j2 = sqrt(1 - cos_j2 * cos_j2);
        j2 = atan2(sin_j2, cos_j2) * 57.3;

        j4 = j_sum * 57.3 - j2 - j3;

        if (j2 >= -30 && j3 >= 0 && j4 >= 0 && j2 <= 90 && j3 <= 180 && j4 <= 180)
        {
            n++;
        }
    }

    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180;
        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
        high = target_z;

        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;

        cos_j3 = ((L * L) + (H * H) - (len_2 * len_2) - (len_3 * len_3)) / (2 * len_2 * len_3);
        if (cos_j3 < -1 || cos_j3 > 1)
            continue;

        sin_j3 = sqrt(1 - cos_j3 * cos_j3);
        j3 = atan2(sin_j3, cos_j3) * 57.3;

        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        if (cos_j2 < -1 || cos_j2 > 1)
            continue;

        sin_j2 = sqrt(1 - cos_j2 * cos_j2);
        j2 = atan2(sin_j2, cos_j2) * 57.3;

        j4 = j_sum * 57.3 - j2 - j3;

        if (j2 >= -30 && j3 >= 0 && j4 >= 0 && j2 <= 90 && j3 <= 180 && j4 <= 180)
        {
            m++;
            if (m == n / 2 || m == (n + 1) / 2)
                break;
        }
    }

    target_angle[0] = (int)j1;
    target_angle[1] = (int)j2;
    target_angle[2] = (int)j3;
    target_angle[3] = (int)j4;

    // 调试打印
    Serial.print("Target X: ");
    Serial.println(target_x);
    Serial.print("Target Y: ");
    Serial.println(target_y);
    Serial.print("Target Z: ");
    Serial.println(target_z);
    Serial.print("Angles: ");
    Serial.print(target_angle[0]);
    Serial.print(", ");
    Serial.print(target_angle[1]);
    Serial.print(", ");
    Serial.print(target_angle[2]);
    Serial.print(", ");
    Serial.println(target_angle[3]);
}

// 单个舵机逐步移动
void RobotArm::gradual_move(int servo_index, int target_angle)
{
    int current_angle = servo_angle[servo_index];
    int step = 1;        // 每次调整的步长
    int delay_time = 20; // 每步的延迟时间

    while (abs(current_angle - target_angle) > 2)
    { // 当误差大于 2 度时继续调整
        if (current_angle < target_angle)
        {
            current_angle += step; // 增加角度
        }
        else
        {
            current_angle -= step; // 减少角度
        }

        // 输出到舵机 //static_cast<int>(angle[i] * 0.6666)
        switch (servo_index)
        {
        case 0:
            servo1.write(static_cast<int>((25+current_angle *0.7778) ));
            break;
        case 1:
            servo2.write(static_cast<int>((55+current_angle *0.7778)));
            break;
        case 2:
            servo3.write(static_cast<int>((20+(current_angle+90) *0.7778) ));
            break;
        case 3:
            servo4.write(static_cast<int>((25+(current_angle+90) *0.7778) ));
            break;
        case 4:
            servo5.write(static_cast<int>((current_angle *0.7778) ));
            break; 
        case 5:
            //servo6.write(static_cast<int>(current_angle * 0.6666)); 
            break;
        }

        delay(delay_time); // 延迟一段时间
    }

    // 更新舵机当前角度
    servo_angle[servo_index] = target_angle;
}

// 逐步移动到目标角度
void RobotArm::move_to_angles(int *target_angles)
{
    for (int i = 0; i < 6; i++)
    {  
        gradual_move(i, target_angles[i]); // 逐步移动每个舵机
    }
}