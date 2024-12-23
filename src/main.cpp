#include <Arduino.h>

// #include "SoftSerialParser.h"
#include "HardSerialParser.h"
#include "Ultrasonic.h"
#include "LineFollower.h"
#include "RobotArm.h"
#include "MaxamWheel.h"
/***************************引脚定义**************************/

// 定义超声波测距模块引脚
const int trigPin = A3; // 触发引脚连接 A3
const int echoPin = A0; // 回声引脚连接 A0

// 定义舵机信号线连接的引脚
const int SERVO1_PIN = 7;
const int SERVO2_PIN = 3;
const int SERVO3_PIN = 5;
const int SERVO4_PIN = 6;
const int SERVO5_PIN = 9;
const int SERVO6_PIN = 8;

/*********************************************************************/
#define DISTANCE_THRESHOLD 15.0 // 距离阈值：15厘米
#define BLOCK_TIME 2000         // 屏蔽时间：2000毫秒
/***************************全局变量定义**************************/
// 循迹机械臂状态：90, 0 ,90 , 90, 180, 0
// 上电状态：90, 0, 0, 0, 180, 0
// 抓取  ：90,10 ,60 ,100, 180, 0
int servoAngle_init[6] = {90, 0, 0, 0, 180, 0};
int servoAngle_follow[6] = {90, 0, 90, 90, 180, 0};
int servoAngle_catch[6] = {90, 10, 60, 100, 180, 0};
volatile float distance;

// 定义轮子的转速数组 (rad/s)
int wheelSpeeds[4] = {-200, -200, -200, -200};

// 创建 SerialParser 对象
// SoftSerialParser softParser(A1, A2);
HardSerialParser hardParser(Serial);
// 创建循迹对象
LineFollower follower(Serial);
MaxamWheel Wheel;
char cmd_return_tmp[64];
// 创建 Ultrasonic 对象
Ultrasonic ultrasonic(trigPin, echoPin);

RobotArm Arm(SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN);

// 定义任务状态
enum TaskState
{
  LINE_FOLLOWER,
  CATCH,
  GO_TO_RED,
  GO_TO_GREEN,
  GOT_TO_BLUE,
  RED_BACK_ZERO,
  GREEN_BACK_ZERO,
  BLUE_BACK_ZERO,
  WRITE,

  TASK_DONE
};
TaskState currentTask; // 当前任务状态

unsigned long task1_time = 0;
const int time1Period = 60;
unsigned long task2_time = 0;
const int time2Period = 60;
unsigned long task3_time = 0;
const int time3Period = 60;
unsigned long task4_time = 0;
const int time4Period = 60;

int color = 0;
int px = 0;
int py = 0;

/*********************************************************************/

/******************************定义函数************************************ */
void readSensors();
void sendOpenMVCommand(uint8_t parameter, unsigned long duration_ms = 0, int interval_ms = 50);

void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available())
  {
    serial.read(); // 丢弃所有可用数据
  }
}
// 检测障碍物函数
bool bolockJudge(int identifierCount);
// 超声波检测标志位函数

void MainUpdate();
void LineFollowTask();
void CatchTask();
void GoToRedTask();
void GoToGreenTask();
void GoToBuleTask();
void RedBackZeroTask();
void GreenBacKZeroTask();
void BuleBackZeroTask();
void WriteTask();

// 根据条件检查任务是否完成
bool checkTask1Complete();
bool checkTask2Complete();
bool checkTask3Complete();

/*************************************************************************** */
// 目标坐标
float target_x = 50;  // x 方向目标点
float target_y = 100; // y 方向目标点
float target_z = 75;  // z 方向目标点

void setup()
{
  Serial.begin(115200);
  // hardParser.setCommandCallback(commandHandler);

  follower.begin();   // 初始化
  ultrasonic.begin(); // 初始化超声波引脚
  Arm.begin();        // 连接舵机到对应引脚
  Arm.set_angle(servoAngle_init);

  currentTask = LINE_FOLLOWER;

  delay(2000);
  Serial.println("Initialization completed");
  task1_time = millis();
}

void loop()
{

  // unsigned long startTime = micros();
  // unsigned long currentTime = millis();
  // Wheel.set_speed(0,300,0);
  follower.followLine(400); // 执行循迹任务
  //  if(Arm.inverse_kinematics(target_x,target_y,target_z)){
  //   Serial.println("sssssss");
  //  }

  //  else{
  //   Serial.println("flase777777777 ");
  //  }
  delay(100);

  readSensors();
  // Arm.set_angle(servoAngle);
  // Arm.move_to_angles(servoAngle);
  //  // 处理串口数据
  //  //softParser.processSerial();
  // hardParser.processSerial();

  // unsigned long endTime = micros();
  // unsigned long executionTime = endTime - startTime;
  // Serial.print("Execution Time: ");
  // Serial.print(executionTime);
  // Serial.println(" microseconds");
}

// ====== 读取传感器数据并处理 ====== //

void readSensors()
{
  float rawDistance = ultrasonic.getDistance();              // 获取未滤波的距离
  float filteredDistance = ultrasonic.getFilteredDistance(); // 获取滤波后的距离

  Serial.print("Raw Distance: ");
  Serial.print(rawDistance);
  Serial.print(" cm, Filtered Distance: ");
  Serial.print(filteredDistance);
  Serial.println(" cm");
}

void MainUpdate()
{
  switch (currentTask)
  {
  case LINE_FOLLOWER:
    LineFollowTask();
    break;
  case CATCH:
    CatchTask();
    break;
  case GO_TO_RED:
    GoToRedTask();
  case GO_TO_GREEN:
    GoToGreenTask();
  case GOT_TO_BLUE:
    GoToBuleTask();
  case RED_BACK_ZERO:
    RedBackZeroTask();
  case GREEN_BACK_ZERO:
    GreenBacKZeroTask();
  case BLUE_BACK_ZERO:
    BuleBackZeroTask();
  case WRITE:
    WriteTask();

    break;
  case TASK_DONE:
    Serial.println("All tasks complete!");
    while (true)
    {
      // 停止循环，或者执行空闲状态逻辑
    }
    break;
  }
}

/*task1 循迹（先加速后减速）进入识别点后停止小车进入任务二

判断是否超过预定时间currentTime - task1_time) / 1000 < time1Period
没有超过就执行快速循迹，反之则慢速循迹
进入慢速之后开启超声波检测，遇到第一个block后停止小车并执行任务二进行抓取任务

需调节参数：time1Period  快速和慢速循迹参数（pid，和vx速度）    超声波检测距离阈值cm
*/
void task1()
{
  Serial.println("Executing Task 1");
  unsigned long currentTime = millis();
  if ((currentTime - task1_time) / 1000 < time1Period)
  {
    follower.followLine(400);
  }
  else
  {
    follower.followLine(200);
    if (checkTask1Complete())
    {
      Serial.println("LineFollowTask  Complete");
      // 停止小车运动
      Wheel.set_speed(0, 0, 0);
      currentTask = CATCH; // 切换到任务 2
    }
  }
}
/*任务 2 的逻辑：调整小车和机械臂进入抓取状态姿态，给openmv发信息进入色块抓取状态并且接收到其发送来的颜色和位置信息,最后实现抓取

先修复小车姿态：Vy和omega，然后读取超声波距离值，并且根据距离值来映射到具体速度下的time，然后右转对准障碍块，
机械臂变换为抓取抓取姿态，然后给openmv发送数据使其改变为颜色抓取模式  并清除串口缓存区数据
读取色块位置和颜色信息，并执行抓取任务
左转，机械臂变换为循迹姿态，并根据颜色进入对应的go to任务

参数：distance和time的映射关系，turn right的time时间信息，openmv的数据和实际物理空间的映射
        修复小车姿态函数
*/
void CatchTask()
{
  Serial.println("Executing CatchTask");

  // 修复小车姿态

  float distance = ultrasonic.getFilteredDistance(); // 获取滤波后的距离

  Wheel.backward(distance);
  Wheel.turn_right();

  Arm.set_angle(servoAngle_catch);

  sendOpenMVCommand(0x02, 1000, 50); // 连续发送模式 2 指令(色块识别），持续 1000ms，每次间隔 50ms
  // if (checkTask2Complete())
  // {
  //   Serial.println("Task 2 Complete");
  //   currentTask = TASK_3; // 切换到任务 3
  // }
  clearSerialBuffer(Serial);

  if (Arm.receiveOpenMVData(color, px, py))
  {
    if (Arm.inverse_kinematics(px, py, 75))
    {
      Wheel.turn_left();
      Arm.set_angle(servoAngle_follow);
      // openmv修改为循迹状态
      sendOpenMVCommand(0x01, 1000, 50); // 连续发送模式 2 指令(色块识别），持续 1000ms，每次间隔 50ms
      clearSerialBuffer(Serial);
      // delay(100);
      switch (color)
      {
      case 1:
        currentTask = GO_TO_RED; // 切换到任务 3
        break;
      case 2:
        currentTask = GO_TO_GREEN;
      case 3:
        currentTask = GOT_TO_BLUE;
      default:
        Serial.println("CatchTask switch failed");
        break;
      }
    }
  }
  {
    Serial.println("CatchTask switch failed:Failed to receive openmv data");
  }
}

/*初始状态：摆正循迹姿态且openmv修改为循迹状态

目标：从标记点出发走到red点（在blockJudge函数判断下执行慢速巡线），
      执行姿态修正函数，然后放下颜色色块，执行go back 任务

在执行blockJudge函数：返回false时执行循迹任务；
返回true时停止小车，然后执行姿态修正函数：修改为放下色块姿态，并发下色块，
执行姿态修正：循迹姿态，然后进行go back（color）任务
记录搬运次数

参数：姿态修正函数，放下色块舵机数据（其二者之间的耦合）
*/
void GoToRedTask()
{
  Serial.println("Executing Task 3");

  if (checkTask3Complete())
  {
    Serial.println("Task 3 Complete");
    currentTask = TASK_DONE; // 所有任务完成
  }
}
/*初始状态：摆正循迹姿态且openmv修改为循迹状态

目标：从标记点出发走到green点（在blockJudge函数判断下执行慢速巡线），
      执行姿态修正函数，然后放下颜色色块，执行go back 任务

在执行blockJudge函数：返回false时执行循迹任务；
返回true时停止小车，然后执行姿态修正函数：修改为放下色块姿态，并发下色块，
执行姿态修正：循迹姿态，然后进行go back（color）任务
记录搬运次数

参数：姿态修正函数，放下色块舵机数据（其二者之间的耦合）
*/
void GoToGreenTask()
{
}

/*初始状态：摆正循迹姿态且openmv修改为循迹状态

目标：从标记点出发走到bule点（在blockJudge函数判断下执行慢速巡线），
      执行姿态修正函数，然后放下颜色色块，执行go back 任务

在执行blockJudge函数：返回false时执行循迹任务；
返回true时停止小车，然后执行姿态修正函数：修改为放下色块姿态，并发下色块，
执行姿态修正：循迹姿态，然后进行go back（color）任务
记录搬运次数

参数：姿态修正函数，放下色块舵机数据（其二者之间的耦合）
*/
void GoToBlueTask()
{
}

// 检查任务 1 是否完成
bool checkTask1Complete()
{
  // float distance = ultrasonic.getDistance();              // 获取未滤波的距离
  float distance = ultrasonic.getFilteredDistance(); // 获取滤波后的距离
  if (distance < 35)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// 检查任务 2 是否完成
bool checkTask2Complete()
{
  // 根据外部条件或内部状态判断
  // 示例：某个传感器触发
  return digitalRead(2) == HIGH; // 假设引脚 2 表示条件满足
}

// 检查任务 3 是否完成
bool checkTask3Complete()
{
  // 根据外部条件或内部状态判断
  // 示例：上位机命令 "TASK_3_DONE"
  return Serial.available() && Serial.readString() == "TASK_3_DONE";
}

void sendOpenMVCommand(uint8_t parameter, unsigned long duration_ms, int interval_ms)
{
  // 检查是否需要连续发送
  if (duration_ms > 0)
  {
    unsigned long startTime = millis(); // 获取当前时间
    while (millis() - startTime < duration_ms)
    {
      // 发送数据帧
      uint8_t frame[3] = {0x5A, parameter, 0xFE}; // 帧头、参数、帧尾
      for (int i = 0; i < 3; i++)
      {
        Serial.write(frame[i]);
      }

      // 可选：调试输出
      Serial.print("Sent Command: ");
      for (int i = 0; i < 3; i++)
      {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      delay(interval_ms); // 发送间隔
    }
  }
  else
  {
    // 单次发送
    uint8_t frame[3] = {0x5A, parameter, 0xFE}; // 帧头、参数、帧尾
    for (int i = 0; i < 3; i++)
    {
      Serial.write(frame[i]);
    }

    // 可选：调试输出
    Serial.print("Sent Command: ");
    for (int i = 0; i < 3; i++)
    {
      Serial.print(frame[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// 用户自定义命令解析回调函数
void commandHandler(char *tokens[], int tokenCount)
{
  // 检查命令格式是否正确
  if (tokenCount < 2)
  {
    Serial.println("Error: Invalid command format.");
    return;
  }

  // 第一个字段作为命令关键字
  char command = tokens[0][0];  // 取第一个字符
  int target = atoi(tokens[1]); // 第二个字段转为目标索引
  int value = atoi(tokens[2]);  // 第三个字段转为目标值

  // 使用 switch-case 处理命令
  switch (command)
  {
  case 'W': // 处理 "W" 命令：设置目标轮速变量值
    if (target == 0)
    {
      wheelSpeeds[0] = value;
      Serial.print("wheelSpeeds[0] set to: ");
      Serial.println(wheelSpeeds[0]);
    }
    else if (target == 1)
    {
      wheelSpeeds[1] = value;
      Serial.print("wheelSpeeds[1] set to: ");
      Serial.println(wheelSpeeds[1]);
    }
    else if (target == 2)
    {
      wheelSpeeds[2] = value;
      Serial.print("wheelSpeeds[2] set to: ");
      Serial.println(wheelSpeeds[2]);
    }

    else if (target == 3)
    {
      wheelSpeeds[3] = value;
      Serial.print("wheelSpeeds[3] set to: ");
      Serial.println(wheelSpeeds[3]);
    }
    else
    {
      Serial.println("Error: Invalid target index.");
    }
    break;

  case 'A': // 处理 "A" 命令：修改舵机值
    if (target == 0)
    {
      servoAngle_init[0] = value;
      Serial.print("servoAngle[0] value: ");
      Serial.println(servoAngle_init[0]);
    }
    else if (target == 1)
    {
      servoAngle_init[1] = value;
      Serial.print("servoAngle[1] set to: ");
      Serial.println(servoAngle_init[1]);
    }
    else if (target == 2)
    {
      servoAngle_init[2] = value;
      Serial.print("servoAngle[2] set to: ");
      Serial.println(servoAngle_init[2]);
    }

    else if (target == 3)
    {
      servoAngle_init[3] = value;
      Serial.print("servoAngle[3] set to: ");
      Serial.println(servoAngle_init[3]);
    }
    else if (target == 4)
    {
      servoAngle_init[4] = value;
      Serial.print("servoAngle[4] set to: ");
      Serial.println(servoAngle_init[4]);
    }
    else if (target == 5)
    {
      servoAngle_init[5] = value;
      Serial.print("servoAngle[5] set to: ");
      Serial.println(servoAngle_init[5]);
    }
    else
    {
      Serial.println("Error: Invalid target index.");
    }
    break;

  case 'R': // 处理 "R" 命令：读取数组的变量值
    if (target == 1)
    {
      // 读取并输出 servoAngle 数组的值
      Serial.println("Servo Angles:");
      for (size_t i = 0; i < sizeof(servoAngle_init) / sizeof(servoAngle_init[0]); i++)
      {
        Serial.print("Servo ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(servoAngle_init[i]);
      }
    }
    else if (target == 2)
    {
      // 读取并输出 wheelSpeeds 数组的值
      Serial.println("Wheel Speeds:");
      for (size_t i = 0; i < sizeof(wheelSpeeds) / sizeof(wheelSpeeds[0]); i++)
      {
        Serial.print("Wheel ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(wheelSpeeds[i]);
      }
    }
    else if (target == 3)
    {
      Serial.print("Left Sensor: ");
      // Serial.print(leftOnLine);
      Serial.print(" | Right Sensor: ");
      // Serial.println(rightOnLine);

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    else
    {
      // 错误处理：target 无效
      Serial.println("Error: Invalid target index.");
    }
    break;

  default: // 未知命令
    Serial.print("Error: Unknown command '");
    Serial.print(command);
    Serial.println("'.");
    break;
  }
}

// 检测障碍物函数:输入需要检测到的障碍块数量。
bool bolockJudge(int identifierCount)
{
  static int passedBlocks = 0;           // 已通过的障碍块数
  static unsigned long blockEndTime = 0; // 超声波屏蔽结束时间

  // 检查当前时间是否仍在屏蔽时间内
  unsigned long currentTime = millis();
  if (currentTime < blockEndTime)
  {
    return false; // 超声波仍处于屏蔽状态，直接返回
  }

  // 获取当前滤波后的超声波测量距离
  float distance = ultrasonic.getFilteredDistance();
  Serial.print("Filtered Distance: ");
  Serial.println(distance);

  // 判断是否小于阈值
  if (distance > 2 && distance < DISTANCE_THRESHOLD)
  {
    passedBlocks++;                       // 记录通过的障碍块
    blockEndTime = millis() + BLOCK_TIME; // 设置屏蔽结束时间
    Serial.print("Obstacle Detected! Passed Blocks: ");
    Serial.println(passedBlocks);
  }

  // 检查是否达到目标标识符数
  if (passedBlocks >= identifierCount)
  {
    Serial.println("Target Identifier Count Reached!");
    passedBlocks = 0; // 重置通过的障碍块数
    blockEndTime = 0; // 重置屏蔽结束时间
    return true;
  }

  return false; // 尚未达到目标标识符数
}
