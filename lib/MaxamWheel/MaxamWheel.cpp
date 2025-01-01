#include "MaxamWheel.h"

MaxamWheel::MaxamWheel()
{
    for (int i = 0; i < 4; i++)
    {
        currentSpeeds[i] = 0; // 初始化当前轮速
        targetSpeeds[i] = 0;
    }
}
// 设置速度的函数 omega为正则向左旋转，即逆时针
void MaxamWheel::set_speed(int Vx, int Vy, int omega)
{
    // 计算每个轮子的目标速度
    targetSpeeds[0] = Vx - Vy - omega * 20; // 前左轮
    targetSpeeds[1] = Vx + Vy + omega * 20; // 前右轮
    targetSpeeds[2] = Vx + Vy - omega * 20; // 后左轮
    targetSpeeds[3] = Vx - Vy + omega * 20; // 后右轮

    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + targetSpeeds[0], 0); // 左前轮
    Serial.println(cmd_return_tmp);

    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 - targetSpeeds[1], 0); // 右前轮
    Serial.println(cmd_return_tmp);

    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + targetSpeeds[2], 0); // 左后轮
    Serial.println(cmd_return_tmp);

    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 - targetSpeeds[3], 0); // 右后轮
    Serial.println(cmd_return_tmp);

    // smoothSetSpeed(targetSpeeds);

    // // 生成并打印指令
    // sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + currentSpeeds[0], 0); // 左前轮
    // Serial.println(cmd_return_tmp);

    // sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 - currentSpeeds[1], 0); // 右前轮
    // Serial.println(cmd_return_tmp);

    // sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + currentSpeeds[2], 0); // 左后轮
    // Serial.println(cmd_return_tmp);

    // sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 - currentSpeeds[3], 0); // 右后轮
    // Serial.println(cmd_return_tmp);
    // // delay(20);
}

// 平滑设置速度
void MaxamWheel::smoothSetSpeed(int targetSpeeds[4])
{
    for (int i = 0; i < 4; i++)
    {
        // 如果当前速度为 0 且目标速度不为 0，直接开始加速
        if (currentSpeeds[i] == 0 && targetSpeeds[i] != 0)
        {
            currentSpeeds[i] = (targetSpeeds[i] > 0) ? SMOOTH_INCREMENT : -SMOOTH_INCREMENT;
            continue; // 跳过后续逻辑，直接进入加速阶段
        }

        // 检查符号是否相反
        if ((currentSpeeds[i] > 0 && targetSpeeds[i] < 0) ||
            (currentSpeeds[i] < 0 && targetSpeeds[i] > 0))
        {
            // 符号相反时，快速减速到 0
            if (currentSpeeds[i] != 0)
            {
                // 直接设置速度接近零的小值，模拟快速刹车
                currentSpeeds[i] += (currentSpeeds[i] > 0) ? -SMOOTH_INCREMENT * 3 : SMOOTH_INCREMENT * 3;

                // 如果速度接近零，直接设置为零
                if (abs(currentSpeeds[i]) < SMOOTH_INCREMENT * 3)
                {
                    currentSpeeds[i] = 0;
                }
                continue; // 当前轮子仍处于减速到零阶段，暂不加速到目标速度
            }
        }

        // 平滑加速或减速
        if (currentSpeeds[i] < targetSpeeds[i])
        {
            // 平滑加速
            currentSpeeds[i] += SMOOTH_INCREMENT;
            if (currentSpeeds[i] > targetSpeeds[i]) // 防止超过目标速度
                currentSpeeds[i] = targetSpeeds[i];
        }
        else if (currentSpeeds[i] > targetSpeeds[i])
        {
            // 平滑减速
            currentSpeeds[i] -= SMOOTH_INCREMENT;
            if (currentSpeeds[i] < targetSpeeds[i]) // 防止低于目标速度
                currentSpeeds[i] = targetSpeeds[i];
        }
    }
}

void MaxamWheel::backward(float distance)
{
    set_speed(0, 0, 0);
    delay(50);
    int time = distance; // 寻找其映射关系
    set_speed(-200, 0, 0);
    delay(time);
    set_speed(0, 0, 0);
    delay(50);
}
void MaxamWheel::turn_left()
{
    set_speed(0, 0, 0);
    delay(20);
    set_speed(0, 0, 200);
    delay(100);
    set_speed(0, 0, 0);
}
void MaxamWheel::turn_right()
{
    set_speed(0, 0, 0);
    delay(20);
    set_speed(0, 0, -200);
    delay(100);
    set_speed(0, 0, 0);
}
void MaxamWheel::corr_offset()
{
    set_speed(0, 0, 0);
    delay(20);
    set_speed(0, 0, -200);
    delay(100);
    set_speed(0, 0, 0);
}