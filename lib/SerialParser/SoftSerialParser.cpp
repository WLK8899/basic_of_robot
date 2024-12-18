#include "SoftSerialParser.h"

// 构造函数：初始化软件串口
SoftSerialParser::SoftSerialParser(int rxPin, int txPin, long baudRate)
    : softSerial(rxPin, txPin) {
    bufferIndex = 0;
    commandCallback = nullptr;
}

// 初始化软件串口
void SoftSerialParser::begin() {
    softSerial.begin(115200);
}

// 设置回调函数
void SoftSerialParser::setCommandCallback(void (*callback)(char *tokens[], int tokenCount)) {
    commandCallback = callback;
}

// 处理串口数据
void SoftSerialParser::processSerial() {
    while (softSerial.available()) {
        char incomingByte = softSerial.read();

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
void SoftSerialParser::parseBuffer() {
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
