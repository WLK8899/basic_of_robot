#include "SerialParser.h"

// 创建 SerialParser 对象
SerialParser serialParser;

// 定义目标变量
int targetVariable1 = 0;
int targetVariable2 = 0;

// 用户自定义命令解析回调函数
void commandHandler(char *tokens[], int tokenCount) {
    // 检查命令格式是否正确
    if (tokenCount < 3) {
        Serial.println("Error: Invalid command format.");
        return;
    }

    // 第一个字段作为命令关键字
    char command = tokens[0][0]; // 取第一个字符
    int target = atoi(tokens[1]);  // 第二个字段转为目标索引
    int value = atoi(tokens[2]);   // 第三个字段转为目标值

    // 使用 switch-case 处理命令
    switch (command) {
        case 'T':  // 处理 "T" 命令：设置目标变量值
            if (target == 1) {
                targetVariable1 = value;
                Serial.print("Target 1 set to: ");
                Serial.println(targetVariable1);
            } else if (target == 2) {
                targetVariable2 = value;
                Serial.print("Target 2 set to: ");
                Serial.println(targetVariable2);
            } else {
                Serial.println("Error: Invalid target index.");
            }
            break;

        case 'R':  // 处理 "R" 命令：读取当前目标变量值
            if (target == 1) {
                Serial.print("Target 1 value: ");
                Serial.println(targetVariable1);
            } else if (target == 2) {
                Serial.print("Target 2 value: ");
                Serial.println(targetVariable2);
            } else {
                Serial.println("Error: Invalid target index.");
            }
            break;

        case 'C':  // 处理 "C" 命令：清零目标变量值
            if (target == 1) {
                targetVariable1 = 0;
                Serial.println("Target 1 cleared.");
            } else if (target == 2) {
                targetVariable2 = 0;
                Serial.println("Target 2 cleared.");
            } else {
                Serial.println("Error: Invalid target index.");
            }
            break;

        default:  // 未知命令
            Serial.print("Error: Unknown command '");
            Serial.print(command);
            Serial.println("'.");
            break;
    }
}

void setup() {
    // 初始化串口解析器
    serialParser.begin();
    // 设置用户自定义命令回调函数
    serialParser.setCommandCallback(commandHandler);

    Serial.println("Serial Parser with Switch-Case Test");
    Serial.println("Send commands like: T 1 32, R 1, C 2");
}

void loop() {
    // 处理串口数据
    serialParser.processSerial();

    // 其他任务
    delay(100);
}
