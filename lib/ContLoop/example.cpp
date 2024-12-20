#include "ContLoop.h"

ContLoop loopController; // 创建任务控制器对象

void setup() {
    Serial.begin(115200);
    loopController.begin(); // 初始化任务控制器
}

void loop() {
    loopController.update(); // 更新任务状态
}
