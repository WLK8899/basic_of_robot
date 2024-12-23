#include "LineFollower.h"

// 构造函数
LineFollower::LineFollower(Stream &serial)
    : serial(serial)
{
}

// 初始化
void LineFollower::begin()
{
    serial.setTimeout(50); // 设置串口超时
}

// 循迹控制任务
void LineFollower::followLine(int Vx)
{
    unsigned long currentTime = millis();
    // 检查是否到达更新间隔
    if (currentTime - lastUpdateTime >= updateInterval)
    {
        lastUpdateTime = currentTime;
        // 计算速度
        int vx = Vx, vy = 0, omega = 0;
        // 接收数据
        int pingyi = 0, pingyi_dir = 0, theta_err = 0, theta_dir = 0;

        if (receiveOpenMVData(pingyi, pingyi_dir, theta_err, theta_dir))
        {
            computeSpeed(pingyi, theta_err, vx, vy, omega);//vx由调用循迹函数时指派

           Wheel.set_speed(vx, vy, omega); // 设置轮子速度
            Serial.print(lastUpdateTime);
            Serial.print(" vx:");
            Serial.print(vx);
            Serial.print(" vy:");
            Serial.print(vy);
            Serial.print(" vz:");
            Serial.println(omega);
        }
        // else{
        //     Wheel.set_speed(0,0,0);
        // }
    }
    else
    {

        // Serial.println("chaoshi : ");
        return;
    }
}

bool LineFollower::receiveOpenMVData(int &pingyi, int &pingyi_dir, int &theta_err, int &theta_dir)
{
    // 从串口读取数据到 buffer

    while (Serial.available() > 0)
    {
        // 搜索帧头
        byte data_by = Serial.read();
        if (bufferIndex == 0 && data_by != 0x5A)
        {
            // Serial.println("222222222222");
            continue; // 如果不是帧头，跳过
        }

        // 存储到缓冲区
        // Serial.println("3333333333");
        buffer[bufferIndex++] = data_by;

        // 如果缓冲区已满，检查是否为完整帧
        if (bufferIndex == 18)
        {
            // 检查帧尾
            if (buffer[17] != 0xFE)
            {
                Serial.println("LineFollower openMV Invalid frame");
                bufferIndex = 0; // 重置缓冲区索引
                return false;
            }

            // 提取数据
            Serial.println("LineFollower openMV is OK");
            int32_t pingyi_zhongxin = (static_cast<int32_t>(buffer[1]) << 24) |
                                      (static_cast<int32_t>(buffer[2]) << 16) |
                                      (static_cast<int32_t>(buffer[3]) << 8) |
                                      static_cast<int32_t>(buffer[4]);

            // int bool_pingyi = buffer[5]; // 中心偏移方向（1 为右，0 为左）
            int32_t bool_pingyi = (static_cast<int32_t>(buffer[5]) << 24) |
                                  (static_cast<int32_t>(buffer[6]) << 16) |
                                  (static_cast<int32_t>(buffer[7]) << 8) |
                                  static_cast<int32_t>(buffer[8]);

            int32_t angle = (static_cast<int32_t>(buffer[9]) << 24) |
                            (static_cast<int32_t>(buffer[10]) << 16) |
                            (static_cast<int32_t>(buffer[11]) << 8) |
                            static_cast<int32_t>(buffer[12]);

            // int bool_angle = buffer[10]; // 旋转方向（1 为右，0 为左）
            int32_t bool_angle = (static_cast<int32_t>(buffer[13]) << 24) |
                                 (static_cast<int32_t>(buffer[14]) << 16) |
                                 (static_cast<int32_t>(buffer[15]) << 8) |
                                 static_cast<int32_t>(buffer[16]);
            // 根据偏移方向（bool_pingyi）设置 pingyi 的正负
            pingyi = (bool_pingyi == 1) ? abs(pingyi_zhongxin) : -abs(pingyi_zhongxin);

            // 根据旋转方向（bool_angle）设置 theta_err 的正负
            theta_err = (bool_angle == 1) ? -abs(angle) : abs(angle);

            // 调试输出
            Serial.print("rho_err: ");
            Serial.println(pingyi);
            Serial.print("theta_err: ");
            Serial.println(theta_err);
            // 处理完成，重置缓冲区
            bufferIndex = 0;
            return true;
        }
    }

    // 未接收到完整帧
    Serial.println("LineFollower openMV Serial flase");
    return false;
}

// 计算速度
void LineFollower::computeSpeed(int rho_err, int theta_err, int &vx, int &vy, int &omega)
{
    // 线性速度控制（PI 控制）
    linear_error_sum += rho_err;
    if (linear_error_sum > 500)
        linear_error_sum = 500; // 防止积分项过大
    if (linear_error_sum < -500)
        linear_error_sum = -500;

    vy = static_cast<int>(Kp_linear * rho_err + Ki_linear * linear_error_sum);

    // 旋转速度控制（P 控制）
    omega = static_cast<int>(Kp_angular * theta_err);

    // 限制速度
    if (vy > MAX_SPEED)
        vy = MAX_SPEED;
    if (vy < -MAX_SPEED)
        vy = -MAX_SPEED;
    if (omega > MAX_ROTATION)
        omega = MAX_ROTATION;
    if (omega < -MAX_ROTATION)
        omega = -MAX_ROTATION;

    // 固定前进速度（假设无特殊需求，保持 0）
    //vx = 350;
}
