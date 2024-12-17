#include "SerialParser.h"

// 构造函数：初始化串口对象和波特率
SerialParser::SerialParser(HardwareSerial &serialPort, long baudRate)
    : serial(serialPort) {
    bufferIndex = 0;
    commandCallback = nullptr; // 默认回调函数为空
}

// 初始化串口
void SerialParser::begin() {
    serial.begin(115200);
}

// 处理串口数据（放在loop中执行）
void SerialParser::processSerial() {
    while (serial.available()) {
        char incomingByte = serial.read();
        
        if (incomingByte == '\n' || incomingByte == '\r') {
            // 如果遇到换行符，解析命令
            buffer[bufferIndex] = '\0'; // 添加字符串结束符
            parseBuffer();
            bufferIndex = 0; // 重置缓冲区索引
        } else if (bufferIndex < MAX_SERIAL_BUFFER - 1) {
            // 存储数据到缓冲区
            buffer[bufferIndex++] = incomingByte;
        }
    }
}

// 设置命令回调函数
void SerialParser::setCommandCallback(void (*callback)(char *tokens[], int tokenCount)) {
    commandCallback = callback;
}

// 内部函数：解析接收到的字符串
void SerialParser::parseBuffer() {
    char *tokens[MAX_TOKENS]; // 字符串分割后的结果
    int tokenCount = 0;

    // 使用 strtok 分割字符串
    char *token = strtok(buffer, " ");
    while (token != NULL && tokenCount < MAX_TOKENS) {
        tokens[tokenCount++] = token;
        token = strtok(NULL, " ");
    }

    // 调用用户定义的回调函数处理解析结果
    if (commandCallback != nullptr) {
        commandCallback(tokens, tokenCount);
    }
}
