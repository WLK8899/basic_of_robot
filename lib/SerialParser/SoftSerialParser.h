#ifndef SOFTSERIALPARSER_H
#define SOFTSERIALPARSER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class SoftSerialParser {
public:
    // 构造函数：允许用户自定义 RX 和 TX 引脚
    SoftSerialParser(int rxPin = A1, int txPin = A2, long baudRate = 9600);

    // 初始化串口
    void begin();

    // 处理串口数据（放在 loop() 中执行）
    void processSerial();

    // 设置用户自定义的命令解析回调函数
    void setCommandCallback(void (*callback)(char *tokens[], int tokenCount));

private:
    SoftwareSerial softSerial; // 软件串口对象
    char buffer[64];           // 接收缓冲区
    int bufferIndex;           // 缓冲区索引
    void (*commandCallback)(char *tokens[], int tokenCount); // 命令回调函数

    void parseBuffer(); // 解析接收的数据
};

#endif
