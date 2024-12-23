#include "ContLoop.h"

// 构造函数
ContLoop::ContLoop() {
    currentTask = TASK_1; // 初始任务为 TASK_1
    
}

// 初始化函数
void ContLoop::begin() {
    // 如果需要额外的初始化逻辑，可以在这里添加
}

// 更新任务状态
void ContLoop::update() {
    switch (currentTask) {
        case TASK_1:
            task1();
            break;
        case TASK_2:
            task2();
            break;
        case TASK_3:
            task3();
            break;
        case TASK_DONE:
            Serial.println("All tasks complete!");
            while (true) {
                // 停止循环，或者执行空闲状态逻辑
            }
            break;
    }
}

// 检查任务 1 是否完成
bool ContLoop::checkTask1Complete() {
    // 根据外部条件或内部状态判断
    // 示例：收到上位机命令 "TASK_1_DONE"
    return Serial.available() && Serial.readString() == "TASK_1_DONE";
}

// 检查任务 2 是否完成
bool ContLoop::checkTask2Complete() {
    // 根据外部条件或内部状态判断
    // 示例：某个传感器触发
    return digitalRead(2) == HIGH; // 假设引脚 2 表示条件满足
}

// 检查任务 3 是否完成
bool ContLoop::checkTask3Complete() {
    // 根据外部条件或内部状态判断
    // 示例：上位机命令 "TASK_3_DONE"
    return Serial.available() && Serial.readString() == "TASK_3_DONE";
}

// 任务 1 的逻辑
void ContLoop::task1() {
    Serial.println("Executing Task 1");
    if (checkTask1Complete()) {
        Serial.println("Task 1 Complete");
        currentTask = TASK_2; // 切换到任务 2
    }
}

// 任务 2 的逻辑
void ContLoop::task2() {
    Serial.println("Executing Task 2");
    if (checkTask2Complete()) {
        Serial.println("Task 2 Complete");
        currentTask = TASK_3; // 切换到任务 3
    }
}

// 任务 3 的逻辑
void ContLoop::task3() {
    Serial.println("Executing Task 3");
    if (checkTask3Complete()) {
        Serial.println("Task 3 Complete");
        currentTask = TASK_DONE; // 所有任务完成
    }
}
