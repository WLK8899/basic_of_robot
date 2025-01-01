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

// 状态转换引脚
const int state_pin2 = A1;
const int state_pin1 = A2;
/*********************************************************************/
#define DISTANCE_THRESHOLD 48 // 距离阈值：15厘米
#define BLOCK_TIME 10000      // 屏蔽时间：2000毫秒

#define RED 4
#define GREEN 3
#define BLUE 2
#define NUMBER 6

int total_num = 0;    // 搬运次数
int total_blocks = 0; // 通过障碍块数量

unsigned long LinetaskTime = 0; // 上次执行任务的时间
bool LinetaskMark = false;

int color_ball;

int noe_color;
/***************************全局变量定义**************************/
/*****************************简单任务循环调度************************** */
static unsigned long startBlockTime = 0;
unsigned long previousMillisTask1 = 0; // 任务 1 上次执行时间
const long intervalTask1 = 50;         // 任务 1 周期：50ms
unsigned long previousMillisTask2 = 0; // 任务 2 上次执行时间
const long intervalTask2 = 100;        // 任务 2 周期：20ms

bool task1InProgress = false; // 标志：任务 1 是否正在执行
/*************************************************************** */
// 循迹机械臂状态：90, 0 ,90 , 90, 180, 0
// 上电状态：90, 0, 0, 0, 180, 0
// 抓取  ：90,10 ,60 ,100, 180, 0
int servoAngle_init[6] = {90, 70, 100, -90, 180, 180};
int servoAngle_follow[6] = {90, 0, 60, 100, 180, 180};
// int servoAngle_follow[6] = {180, 10, 120, 0, 180, 0};
int servoChange[6] = {90, 0, 0, 80, 0, 0};
int servoAngle_catch[6] = {0, 10, 90, 50, 180, 180};
int servoAngle_down[6];
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
float filteredDistance = 300;
RobotArm Arm(SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN);

// 定义任务状态
enum TaskState
{
  LINE_FOLLOWER,
  CATCH,
  GO_TO_RED,
  GO_TO_GREEN,
  GO_TO_BLUE,
  RED_BACK_ZERO,
  GREEN_BACK_ZERO,
  BLUE_BACK_ZERO,
  WRITE,
  RELEASE,

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

int px = 0;
int py = 0;

/*********************************************************************/

/******************************定义函数************************************ */
void readSensors();
// void sendOpenMVCommand(uint8_t parameter, unsigned long duration_ms = 1000, int interval_ms = 50);

void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available())
  {
    serial.read(); // 丢弃所有可用数据
  }
}
// 检测障碍物函数
bool blockJudge(int identifierCount);
// 姿态修正函数
void corr_pos(LineFollower &follower, int vx = 0, unsigned long totalTime = 2000, unsigned long cycleTime = 30);

void MainUpdate();
void LineFollowTask();
void CatchTask();
void GoToColorTask(int color);
void ColorBackZeroTask(int color);
void WriteTask();
void ReleaseTask();
void commandHandler(char *tokens[], int tokenCount);
/*************************************************************************** */
// 目标坐标
float target_x = 0;   // x 方向目标点
float target_y = 240; // y 方向目标点
float target_z = 120; // z 方向目标点

int col;
int x;
int y;
void setup()
{
  Serial.begin(115200);
  hardParser.setCommandCallback(commandHandler);
  pinMode(state_pin1, OUTPUT);
  pinMode(state_pin2, OUTPUT);
  pinMode(A5, INPUT_PULLUP);

  digitalWrite(state_pin1, LOW);
  digitalWrite(state_pin2, LOW);
  // digitalWrite(state_pin1, HIGH);
  // digitalWrite(state_pin2, HIGH);
  follower.begin();   // 初始化
  ultrasonic.begin(); // 初始化超声波引脚
  Arm.begin();        // 连接舵机到对应引脚
                      // Arm.set_angle(servoAngle_follow);
  Arm.set_angle(servoAngle_follow);
  // Arm.set_angle(servoAngle_init);
  currentTask = LINE_FOLLOWER;

  // ultrasonic.corrDistance();

  delay(1000);
  Serial.println("Initialization completed");
}

void loop()
{
  // // Arm.Catch();
  //  hardParser.processSerial();
  // Arm.set_angle(servoAngle_init);
  // //  Arm.Catch();
  // delay(1000);
  // Arm.set_angle(servoAngle_catch);
  // Arm.Release();

  // delay(1000);
  // Arm.set_angle(servoAngle_init);
  //  Arm.Release();
  //  delay(1000);
  // digitalWrite(state_pin1,LOW);
  // digitalWrite(state_pin2,HIGH);
  // unsigned long startTime = micros();
  // follower.followLine();
  // ReleaseTask();
  // CatchTask();
  unsigned long currentMillis = millis();
  // 执行主任务
  if (currentMillis - previousMillisTask1 >= intervalTask1)
  {
    previousMillisTask1 = currentMillis; // 更新主任务 执行时间
    task1InProgress = true;              // 主任务 开始执行

    // LineFollowTask(); // 执行主任务 相关操作
    MainUpdate();
    // follower.followLine();
    // CatchTask();
    task1InProgress = false; // 主任务 执行完毕
  }
  // 执行执行  超声波读取 任务，但如果 主任务 正在执行则跳过
  if (!task1InProgress && (currentMillis - previousMillisTask2 >= intervalTask2))
  {
    // 执行  超声波读取 任务
    previousMillisTask2 = currentMillis;

  readSensors(); // 执行  超声波读取 任务
  }
  // bool a = Arm.inverse_kinematics(target_x,target_y,target_z,1);
  // follower.followLine();
  // Wheel.set_speed(300,0,0);

  // unsigned long endTime = micros();
  // unsigned long executionTime = endTime - startTime;
  // Serial.print("Execution Time: ");
  // Serial.print(executionTime);
  // Serial.println(" microseconds");
}

// ====== 读取传感器数据并处理 ====== //

void readSensors()
{
  // float rawDistance = ultrasonic.getDistance();              // 获取未滤波的距离
  filteredDistance = ultrasonic.getFilteredDistance(); // 获取滤波后的距离

  // Serial.print("Raw Distance: ");
  // Serial.print(rawDistance);
  Serial.print(" cm, Filtered Distance: ");
  Serial.print(filteredDistance);
  Serial.println(" cm");
}

void MainUpdate()
{
  static int lastTask = -1; // 假设任务编号为非负整数
  if (currentTask != lastTask)
  {
    Serial.print("Updating to task: ");
    Serial.println(currentTask);
    lastTask = currentTask;
  }

  switch (currentTask)
  {
  case LINE_FOLLOWER:
    LineFollowTask();
    break;
  case CATCH:
    CatchTask();
    break;
  case GO_TO_RED:
    GoToColorTask(RED);
    break;
  case GO_TO_GREEN:
    GoToColorTask(GREEN);
    break;
  case GO_TO_BLUE:
    GoToColorTask(BLUE);
    break;
  case RED_BACK_ZERO:
    ColorBackZeroTask(NUMBER - RED);
    break;
  case GREEN_BACK_ZERO:
    ColorBackZeroTask(NUMBER - GREEN);
    break;
  case BLUE_BACK_ZERO:
    ColorBackZeroTask(NUMBER - BLUE);
    break;
  case RELEASE:
    ReleaseTask();
    break;
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
  default:
    Serial.println("MainUpdate: Unknown task!");
    break;
  }
}

/*task1 循迹（先加速后减速）进入识别点后停止小车进入任务二

判断是否超过预定时间currentTime - task1_time) / 1000 < time1Period
没有超过就执行快速循迹，反之则慢速循迹
进入慢速之后开启超声波检测，遇到第一个block后停止小车并执行任务二进行抓取任务

需调节参数：time1Period  快速和慢速循迹参数（pid，和vx速度）    超声波检测距离阈值cm
*/
void LineFollowTask()
{
  Serial.println("Executing LineFollowTask");
  // 记录程序启动的时间，只需执行一次
  if (!LinetaskMark)
  {
    LinetaskTime = millis();
    LinetaskMark = true;
  }

  // 在前40秒内执行followLine()，但不执行 if 判断
  if (millis() - LinetaskTime < 1000L * 40)
  {
    follower.followLine();
    return; // 在40秒内仅执行 followLine()，不进行 if 判断
  }
  if (filteredDistance < 45)
  {
    Serial.println("LineFollowTask Complete");

    // 停止小车运动
    delay(10);
    Wheel.set_speed(0, 0, 0);
    delay(10);

    // 切换到下一任务（任务 2：CATCH）
    currentTask = CATCH;
    Wheel.set_speed(0, 0, 0);
    // digitalWrite(state_pin1, HIGH);
    // digitalWrite(state_pin2, LOW); // 进入姿态校准模式(vx = 0, vy omega校准)
    // delay(200);
    clearSerialBuffer(Serial); // 清空串口缓冲区
    Wheel.set_speed(0, 0, 0);
    delay(2500);
    return;
  }
  // 每次都执行循迹
  follower.followLine();

  // 如果 OpenMV 指令已经发送过，检查任务是否完成
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

  // 静态变量，用于标记是否已完成初始设置（修正姿态、设置机械臂姿态、切换抓取模式）
  static bool catchinit = false;

  // 初始化部分，仅执行一次
  if (!catchinit)
  {
    // 设置机械臂到抓取姿态
    Arm.set_angle(servoChange);
    delay(500);
    Arm.set_angle(servoAngle_catch);

    // 向 OpenMV 发送指令，进入色块抓取模式
    digitalWrite(state_pin1, HIGH);
    digitalWrite(state_pin2, HIGH); // 进入色块抓取模式
    clearSerialBuffer(Serial);      // 清空串口缓冲区
    delay(2500);
    catchinit = true; // 标记已初始化
    return;           // 直接退出，等待下一次执行
  }

  // 检查是否检测到色块
  if (digitalRead(A5) == LOW)
  {
    Wheel.set_speed(0, 0, 0);
    if (Arm.receiveOpenMVData(color_ball, px, py)) // 检查 OpenMV 返回的数据
    {
      // Arm.Coordinate_mapping(px, py);
      Arm.Catch();
      // 根据色块位置进行逆运动学解算
      delay(1000);
      Arm.set_angle(servoAngle_follow); // 恢复机械臂到循迹状态
      digitalWrite(state_pin1, LOW);
      digitalWrite(state_pin2, HIGH);
      clearSerialBuffer(Serial);
      delay(2000);

      // 向 OpenMV 发送指令，切换回循迹模式
      // sendOpenMVCommand(0x02, 1000, 50);

      // 根据颜色切换到对应任务
      switch (color_ball)
      {
      case RED:
        currentTask = GO_TO_RED;
        break;
      case GREEN:
        currentTask = GO_TO_GREEN;
        break;
      case BLUE:
        currentTask = GO_TO_BLUE;
        break;
      default:
        Serial.println("CatchTask: Invalid color detected");
        break;
      }
      catchinit = false; // 重置初始化标记，为下一次任务做准备
      return;            // 任务完成，退出函数
    }
  }
  else
  {
    // 如果未检测到色块，则继续循迹（后退到目标点，色块在视觉中央）
    follower.followLine();
  }
}

void ReleaseTask()
{
  Serial.println("Executing ReleaseTask");

  // 静态变量，用于标记是否已完成初始设置（修正姿态、设置机械臂姿态、切换抓取模式）
  static bool releaseinit = false;

  // 初始化部分，仅执行一次
  if (!releaseinit)
  {
    Wheel.set_speed(0, 0, 0);
    // 设置机械臂到抓取姿态
    Arm.set_angle(servoAngle_catch);

    // 向 OpenMV 发送指令，进入色块抓取模式
    // digitalWrite(state_pin1, HIGH);
    // digitalWrite(state_pin2, LOW); // 进入色块抓取模式
    // clearSerialBuffer(Serial);     // 清空串口缓冲区
    delay(1000);
    releaseinit = true; // 标记已初始化
    return;             // 直接退出，等待下一次执行
  }

  // 检查是否检测到色块
  Wheel.backward(3000);
  Arm.Release();
  delay(1000);
  Arm.set_angle(servoAngle_follow);
  digitalWrite(state_pin1, LOW);
  digitalWrite(state_pin2, HIGH);
  // 根据颜色切换到对应任务
  switch (color_ball)
  {
  case RED:
    currentTask = RED_BACK_ZERO;
  case GREEN:
    currentTask = GREEN_BACK_ZERO;
    break;
  case BLUE:
    currentTask = BLUE_BACK_ZERO;
    break;
  default:
    Serial.println("ReleaseTask: Invalid color detected");
    break;
  }

  releaseinit = false; // 重置初始化标记，为下一次任务做准备
  return;              // 任务完成，退出函数
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
void GoToColorTask(int color)
{
  Serial.println("Executing Task: GoToRedTask");

  // 如果未到达目标（通过 RED 的障碍物数未满足要求）
  if (!blockJudge(color))
  // if(1)
  {
    follower.followLine(); // 执行循迹任务
    return;                // 未完成任务，直接返回
  }

  // 已完成任务逻辑
  Wheel.set_speed(0, 0, 0); // 停止轮子运动
  delay(100);               // 稳定小车位置

  // Wheel.set_speed(0, 0, 400);
  // delay(50000);
  // 操作机械臂进行抓取操作
  Arm.set_angle(servoAngle_catch); // 机械臂调整到抓取状态

  // 记录搬运次数  并且 切换任务为 RED_BACK_ZERO
  total_num++;
  color_ball = color;
  currentTask = RELEASE;
  // 根据颜色切换到对应任务
}
/*
在blockJudge(color)判断下执行慢速循迹到达识别点
到达之后判读total_num：如果大于等于3，则执行writeTask
                      反之执行Catch
*/
void ColorBackZeroTask(int color)
{
  // 检查是否通过了足够的障碍块
  if (!blockJudge(color))
  {
    follower.followLine(); // 如果未完成，继续循迹
    return;                // 提前退出，避免后续逻辑执行
  }

  // 判断总数量并切换到对应任务
  Wheel.set_speed(0, 0, 0);
  delay(100);
  currentTask = (total_num < 3) ? CATCH : WRITE;
}

void WriteTask()
{
  Wheel.set_speed(0, 0, 500);
}

// void sendOpenMVCommand(uint8_t parameter, unsigned long duration_ms, int interval_ms)
// {
//   // 检查是否需要连续发送
//   if (duration_ms > 0)
//   {
//     unsigned long startTime = millis(); // 获取当前时间
//     while (millis() - startTime < duration_ms)
//     {
//       // 发送数据帧
//       uint8_t frame[3] = {0x5A, parameter, 0xFE}; // 帧头、参数、帧尾
//       for (int i = 0; i < 3; i++)
//       {
//         Serial.write(frame[i]);
//       }

//       // 可选：调试输出
//       Serial.print("Sent Command: ");
//       for (int i = 0; i < 3; i++)
//       {
//         Serial.print(frame[i], HEX);
//         Serial.print(" ");
//       }
//       Serial.println();

//       delay(interval_ms); // 发送间隔
//     }
//   }
//   else
//   {
//     // 单次发送
//     uint8_t frame[3] = {0x5A, parameter, 0xFE}; // 帧头、参数、帧尾
//     for (int i = 0; i < 3; i++)
//     {
//       Serial.write(frame[i]);
//     }

//     // 可选：调试输出
//     Serial.print("Sent Command: ");
//     for (int i = 0; i < 3; i++)
//     {
//       Serial.print(frame[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();
//   }
// }

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

// 检测障碍物函数: 输入需要检测到的障碍块数量。
bool blockJudge(int identifierCount)
{
  // 通过辅助函数访问静态变量
  unsigned long currentTime = millis();

  if (ultrasonic.getDistance() <= 45.0 && ultrasonic.getDistance() >= 15)
  {
    if (startBlockTime < currentTime)
    {
      total_blocks++;
      startBlockTime = currentTime + BLOCK_TIME;
      if (total_blocks < identifierCount)
      {
        // follower.followLine();
        return false;
      }
      else
      {
        total_blocks = 0;
        // startBlockTime = 0;
        return true;
      }
    }
    else
      return false;
  }
  else
    return false;
}

void corr_pos(LineFollower &follower, int vx, unsigned long totalTime, unsigned long cycleTime)
{
  unsigned long startTime = millis(); // 记录起始时间
  unsigned long lastCycleTime = 0;    // 上一次任务执行的时间点

  while (millis() - startTime < totalTime)
  { // 循环执行2秒
    unsigned long currentTime = millis();

    // 检查是否到了执行周期
    if (currentTime - lastCycleTime >= cycleTime)
    {
      lastCycleTime = currentTime; // 更新上一次执行的时间点

      // 执行循迹任务
      follower.followLine();
    }

    // 保持其余时间的执行流畅
    delay(1); // 小延迟，避免过高的空循环占用CPU
  }
}
