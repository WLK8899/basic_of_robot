#ifndef CONT_LOOP_H
#define CONT_LOOP_H

#include <Arduino.h>
#include "Ultrasonic.h"
#include "LineFollower.h"
#include "RobotArm.h"
#include "MaxamWheel.h"

// 定义任务状态
enum TaskState
{
    TASK_1,
    TASK_2,
    TASK_3,
    TASK_DONE
};

class ContLoop
{
private:
    TaskState currentTask; // 当前任务状态

    // 私有任务函数
    void task1();
    void task2();
    void task3();

    // 根据条件检查任务是否完成
    bool checkTask1Complete();
    bool checkTask2Complete();
    bool checkTask3Complete();

public:
    ContLoop();    // 构造函数
    void begin();  // 初始化函数（可用于设置状态）
    void update(); // 更新任务状态
};

#endif
