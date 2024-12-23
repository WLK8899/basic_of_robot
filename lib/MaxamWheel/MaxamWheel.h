#ifndef MaxamWheel_H
#define MaxamWheel_H

#include <Arduino.h>

class MaxamWheel
{

public:
    MaxamWheel();

    void set_speed(int Vx, int Vy, int omega);
    
    void backward(float distance);
    void turn_right();
    void turn_left();
    void corr_offset();
private:
    // 麦克纳姆轮速度数组
    int targetSpeeds[4];
    int currentSpeeds[4];

    char cmd_return_tmp[64];
    const int SMOOTH_INCREMENT = 10; // 平滑速度增量

    void smoothSetSpeed(int targetSpeeds[4]);
};

#endif
