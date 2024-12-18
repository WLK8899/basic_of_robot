#include "HardSerialParser.h"

// 构造函数：初始化硬件串口
HardSerialParser::HardSerialParser(HardwareSerial &serialPort, long baudRate)
    : serial(serialPort), baudRate(baudRate) {
    bufferIndex = 0;
    commandCallback = nullptr;
}

// 初始化硬件串口
void HardSerialParser::begin() {
    serial.begin(baudRate);
}

// 设置回调函数
void HardSerialParser::setCommandCallback(void (*callback)(char *tokens[], int tokenCount)) {
    commandCallback = callback;
}

// 处理串口数据
void HardSerialParser::processSerial() {
    while (serial.available()) {
        char incomingByte = serial.read();

        if (incomingByte == '\n' || incomingByte == '\r') {
            buffer[bufferIndex] = '\0';
            parseBuffer();
            bufferIndex = 0;
        } else if (bufferIndex < sizeof(buffer) - 1) {
            buffer[bufferIndex++] = incomingByte;
        }
    }
}

// 解析接收数据
void HardSerialParser::parseBuffer() {
    char *tokens[5];
    int tokenCount = 0;

    char *token = strtok(buffer, " ");
    while (token != NULL && tokenCount < 5) {
        tokens[tokenCount++] = token;
        token = strtok(NULL, " ");
    }

    if (commandCallback != nullptr) {
        commandCallback(tokens, tokenCount);
    }
}
