#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <thread>
#include "CAN_comm.h"
#include "config.h"


#define myID 0x10

MIT devicesState[4];

uint32_t sendNum; // for test send speed
uint32_t recNum;

MIT MITCtrlParam;


uint16_t sendCounter = 0;
bool motorEnable = true;
int receivedNumber = 0;
uint64_t prev_ts = 0;
float t = 0.0f;
float targetJointAngle = 0.0f; // 目标关节角度，可在运行时输入修改

namespace {
uint64_t micros_steady(){
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
}


void setup() {
  std::printf("SF Motor Control (Jetson) start\n");
  CANInit();
  enable(0x01);//使能ID为0X01的电机
  prev_ts = micros_steady();
  t = 0.0f;
}

uint16_t printCount = 0;
uint16_t recCount = 0;


void loop() {

  recCANMessage();

  // 检查是否有新的关节角度输入（每100次循环检查一次，避免频繁检查）
  static uint16_t inputCheckCount = 0;
  if(++inputCheckCount >= 1000){
    inputCheckCount = 0;
    float newAngle;
    if(std::scanf("%f", &newAngle) == 1){
      targetJointAngle = newAngle;
      std::printf("已更新目标关节角度: %.3f rad\n", newAngle);
    }
  }

  static int IDswitch = 0x01;
  uint64_t current_ts = micros_steady();

  /*
 * 函数功能：根据时间差更新控制参数，并发送MIT命令。
 * 参数说明：
 *   - rent_ts: 当前时间戳
 *   - prev_ts: 上一次记录的时间戳
 *   - t: 时间变量，用于计算正弦和余弦函数
 *   - MITCtrlParam: 控制参数结构体，包含位置、速度、位置增益、速度增益和扭矩等参数
 *   - IDswitch: 用于切换ID的计数器
 * 返回值：无
 */
  if(current_ts - prev_ts >= 1000){//1ms
    // 更新时间变量t，增加1毫秒
    t+=0.001;
    // 设置控制参数中的目标位置、目标速度和位置增益、速度增益和扭矩  
    MITCtrlParam.pos = targetJointAngle; 
    MITCtrlParam.vel = 0;
    MITCtrlParam.kp = 0.5;
    MITCtrlParam.kd = 0.3;
    MITCtrlParam.tor = 0;
    // 更新上一次记录的时间戳
    prev_ts = current_ts;

    // IDswitch++;
    // 如果IDswitch超过0x04，则重置为0x01
    // if(IDswitch > 0x04){
    //   IDswitch = 0x01;
    // }
    sendMITCommand(IDswitch, MITCtrlParam);// 发送MIT命令

    printCount++;
    if(printCount >= 100){ //
      printCount = 0;
      // 仅当IDswitch为0x01时，分别打印发送出去和接收电机状态的pos和vel值
      if(IDswitch == 0x01){
        std::printf("%.2f,%.2f,%.2f,%.2f\n", MITCtrlParam.pos, MITCtrlParam.vel,devicesState[IDswitch - 1].pos, devicesState[IDswitch - 1].vel);
      }      
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

int main(){
  setup();
  
  while(true){
    loop();
  }
  disable(0x01);
  return 0;
}

