void sendCommand(int motor, int speed) {
    char cmd_return_tmp[64];
    sprintf(cmd_return_tmp, "#%03dP%04dT0000!", motor, 1500 + speed);
    Serial.println(cmd_return_tmp);
}

void rotation(int degree) {
    // 定义旋转速度
    const int ROTATION_SPEED = 465; 
    const int TIME_PER_DEGREE = 8; // 单位延时时间(ms/degree)

    int rotationDirection = (degree > 0) ? ROTATION_SPEED : -ROTATION_SPEED; // 判断旋转方向

    // 设置电机速度
    sendCommand(6, rotationDirection);
    sendCommand(7, rotationDirection);
    sendCommand(8, rotationDirection);
    sendCommand(9, rotationDirection);

    // 计算旋转时间
    delay(TIME_PER_DEGREE * abs(degree));

    // 停止电机
    sendCommand(6, 0);
    sendCommand(7, 0);
    sendCommand(8, 0);
    sendCommand(9, 0);
}

// void set_speed(int *wheelSpeeds)
// {
//   sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 6, 1500 + wheelSpeeds[0], 0); // 组合指令
//   Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
//   delay(20);
//   sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 7, 1500 - wheelSpeeds[0], 0); // 组合指令
//   Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
//   delay(20);
//   sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 8, 1500 + wheelSpeeds[0], 0); // 组合指令
//   Serial.println(cmd_return_tmp);
//   delay(20);                                                                // 解析ZMotor指令-左电机正向
//   sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", 9, 1500 - wheelSpeeds[0], 0); // 组合指令
//   Serial.println(cmd_return_tmp);                                           // 解析ZMotor指令-左电机正向
//   delay(20);
// }
void readSensors()
{
  // 使用digitalRead读取传感器状态
  leftOnLine = digitalRead(leftSensorPin) ; // 黑线为低电平
  rightOnLine = digitalRead(rightSensorPin) ;
  distance = ultrasonic.getDistance(); // 获取距离值

  // 调试输出
  Serial.print("Left Sensor: ");
  Serial.print(leftOnLine);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightOnLine);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}