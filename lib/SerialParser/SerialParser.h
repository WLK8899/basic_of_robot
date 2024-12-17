#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include <Arduino.h>

// 定义接收字符串的最大长度
#define MAX_SERIAL_BUFFER 64
#define MAX_TOKENS 5       // 最大支持分割成5个字段

class SerialParser {
public:
    // 构造函数
    SerialParser(HardwareSerial &serialPort = Serial, long baudRate = 115200);

    // 初始化串口
    void begin();

    // 处理串口数据（放在loop中执行）
    void processSerial();

    // 用户自定义的命令解析函数
    void setCommandCallback(void (*callback)(char *tokens[], int tokenCount));

private:
    HardwareSerial &serial;       // 串口对象
    char buffer[MAX_SERIAL_BUFFER]; // 接收缓冲区
    int bufferIndex;              // 缓冲区当前索引
    void (*commandCallback)(char *tokens[], int tokenCount); // 命令回调函数

    // 内部函数：解析接收到的字符串
    void parseBuffer();
};

#endif
