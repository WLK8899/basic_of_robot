#include <Arduino.h>

void sendCommand(String command)
{
    // 通过硬件串口发送字符串指令
    Serial.print(command);
    Serial.print('\r'); // 添加回车符结束指令（如果协议需要）
}
void modifyMotorID()
{
    // 根据文档发送修改ID的指令
    // 修改双路ID为6和8：指令格式为 `#255PID006+008!`
    String command = "#255PID006+008!";
    sendCommand(command);
}

void setup()
{
    delay(1000);
    // 修改双路ID为6和8
    modifyMotorID();
}

void loop()
{
}