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
void LineFollower::followLine()
{
   // unsigned long currentTime = millis();
    // 检查是否到达更新间隔
    // if (currentTime - lastUpdateTime >= updateInterval)
    // {
    //     lastUpdateTime = currentTime;
    // 计算速度
    int32_t vx = 0, vy = 0, omega = 0;
    // 接收数据
    if (receiveOpenMVData(vx, vy, omega))
    {
        // computeSpeed(vx, vy, omega); // vx由调用循迹函数时指派

        Wheel.set_speed(vx, vy, omega); // 设置轮子速度
        // Serial.print(lastUpdateTime);
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
    // }
    // else
    // {

    //     // Serial.println("chaoshi : ");
    //     return;
    // }
}

bool LineFollower::receiveOpenMVData(int32_t &VX, int32_t &VY, int32_t &OMEGA)
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
        if (bufferIndex == 14)
        {
            // 检查帧尾
            if (buffer[13] != 0xFE)
            {
                Serial.println("LineFollower openMV Invalid frame");
                bufferIndex = 0; // 重置缓冲区索引
                return false;
            }

            // 提取数据
            Serial.println("LineFollower openMV is OK");
            VX = (static_cast<int32_t>(buffer[1]) << 24) |
                 (static_cast<int32_t>(buffer[2]) << 16) |
                 (static_cast<int32_t>(buffer[3]) << 8) |
                 static_cast<int32_t>(buffer[4]);

            // int bool_pingyi = buffer[5]; // 中心偏移方向（1 为右，0 为左）
            VY = (static_cast<int32_t>(buffer[5]) << 24) |
                 (static_cast<int32_t>(buffer[6]) << 16) |
                 (static_cast<int32_t>(buffer[7]) << 8) |
                 static_cast<int32_t>(buffer[8]);

            OMEGA = (static_cast<int32_t>(buffer[9]) << 24) |
                    (static_cast<int32_t>(buffer[10]) << 16) |
                    (static_cast<int32_t>(buffer[11]) << 8) |
                    static_cast<int32_t>(buffer[12]);

            // 调试输出
            Serial.print(" vx: ");
            Serial.println(VX);
            Serial.print(" vy: ");
            Serial.println(VY);
            Serial.print(" omega: ");
            Serial.println(OMEGA);

            // 处理完成，重置缓冲区
            bufferIndex = 0;
            return true;
        }
    }

    // 未接收到完整帧
    Serial.println("LineFollower openMV Serial flase");
    return false;
}

// 计算速度（PID 控制）
void LineFollower::computeSpeed(int rho_err, int theta_err, int &vx, int &vy, int &omega)
{
    // 获取当前时间
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // 计算时间间隔（秒）
    if (dt <= 0)
        dt = 0.01; // 防止 dt 为 0

    // --- 线性速度 PID 控制 ---
    // 比例误差
    float linear_error = rho_err;

    // 积分误差
    linear_error_sum += linear_error * dt;
    if (linear_error_sum > 300)
        linear_error_sum = 300; // 防止积分过大
    if (linear_error_sum < -300)
        linear_error_sum = -300;

    // 微分误差
    float linear_error_rate = (linear_error - linear_error_prev) / dt;

    // 计算 PID 输出
    vy = static_cast<int>(
        Kp_linear * linear_error +     // 比例项
        Ki_linear * linear_error_sum + // 积分项
        Kd_linear * linear_error_rate  // 微分项
    );

    // 更新上次线性误差
    linear_error_prev = linear_error;

    // --- 旋转速度 PID 控制 ---
    // 比例误差
    float angular_error = theta_err;

    // 积分误差
    angular_error_sum += angular_error * dt;
    if (angular_error_sum > 300)
        angular_error_sum = 300; // 防止积分过大
    if (angular_error_sum < -300)
        angular_error_sum = -300;

    // 微分误差
    float angular_error_rate = (angular_error - angular_error_prev) / dt;

    // 计算 PID 输出
    omega = static_cast<int>(
        Kp_angular * angular_error +     // 比例项
        Ki_angular * angular_error_sum + // 积分项
        Kd_angular * angular_error_rate  // 微分项
    );

    // 更新上次旋转误差
    angular_error_prev = angular_error;

    // --- 限制速度 ---
    if (vy > MAX_SPEED)
        vy = MAX_SPEED;
    if (vy < -MAX_SPEED)
        vy = -MAX_SPEED;
    if (omega > MAX_ROTATION)
        omega = MAX_ROTATION;
    if (omega < -MAX_ROTATION)
        omega = -MAX_ROTATION;

    // 固定前进速度（假设需要）
    // vx = 0; // 如果 vx 需要调整，可以根据需求修改

    // 更新上次更新时间
    lastUpdateTime = currentTime;
}
