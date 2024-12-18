#ifndef HARDSERIALPARSER_H
#define HARDSERIALPARSER_H

#include <Arduino.h>

class HardSerialParser {
public:
    // 构造函数：传入 HardwareSerial 对象和波特率
    HardSerialParser(HardwareSerial &serialPort = Serial, long baudRate = 9600);

    // 初始化串口
    void begin();

    // 处理串口数据（放在 loop() 中执行）
    void processSerial();

    // 设置用户自定义的命令解析回调函数
    void setCommandCallback(void (*callback)(char *tokens[], int tokenCount));

private:
    HardwareSerial &serial;
    long baudRate;
    char buffer[64];
    int bufferIndex;
    void (*commandCallback)(char *tokens[], int tokenCount);

    void parseBuffer();
};

#endif
