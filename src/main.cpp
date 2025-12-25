#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <thread>
#include "CAN_comm.h"
#include "config.h"

MIT devicesState[4];

uint32_t sendNum; // for testing send speed
uint32_t recNum;

MIT MITCtrlParam;

uint16_t sendCounter = 0;
bool motorEnable = true;
int receivedNumber = 0;
uint64_t prev_ts = 0;
float t = 0.0f;
float targetJointAngle = 0.0f; // Target joint angle (can be modified at runtime via input)

namespace {
uint64_t micros_steady(){
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
}

void setup() {
  std::printf("SF Motor Control (Jetson) start\n");
  CANInit();
  enable(0x01); // Enable motor with ID 0x01  <- Change ID to control different motors
  prev_ts = micros_steady();
  t = 0.0f;
}

uint16_t printCount = 0;
uint16_t recCount = 0;

void loop() {

  recCANMessage();

  // Check for new joint angle input
  // (Check once every 1000 loops to avoid frequent blocking input calls)
  static uint16_t inputCheckCount = 0;
  if(++inputCheckCount >= 1000){
    inputCheckCount = 0;
    float newAngle;
    if(std::scanf("%f", &newAngle) == 1){
      targetJointAngle = newAngle;
      std::printf("Target joint angle updated: %.3f rad\n", newAngle);
    }
  }

  static int IDswitch = 0x01; // <- Change ID to control different motors
  uint64_t current_ts = micros_steady();

  /*
   * Function:
   *   Update control parameters based on time difference and send MIT command.
   *
   * Parameters:
   *   - current_ts: current timestamp
   *   - prev_ts   : previous timestamp
   *   - t         : time variable used for sine/cosine calculations
   *   - MITCtrlParam:
   *       Control parameter structure including position, velocity,
   *       position gain (Kp), velocity gain (Kd), and torque
   *   - IDswitch  : motor ID selector
   *
   * Return:
   *   None
   */
  if(current_ts - prev_ts >= 1000){ // 1 ms control period
    // Update time variable (increase by 1 ms)
    t += 0.001;

    // Set control parameters:
    // target position, target velocity, position gain, velocity gain, and torque
    MITCtrlParam.pos = targetJointAngle;
    MITCtrlParam.vel = 0;
    MITCtrlParam.kp  = 0.5;
    MITCtrlParam.kd  = 0.3;
    MITCtrlParam.tor = 0;

    // Update previous timestamp
    prev_ts = current_ts;

    // IDswitch++;
    // If IDswitch exceeds 0x04, reset it to 0x01
    // if(IDswitch > 0x04){
    //   IDswitch = 0x01;
    // }

    sendMITCommand(IDswitch, MITCtrlParam); // Send MIT command

    printCount++;
    if(printCount >= 100){
      printCount = 0;
      // Only print when IDswitch is 0x01
      // Print commanded position/velocity and actual motor position/velocity
      if(IDswitch == 0x01){
        std::printf( "[CMD] pos: %6.3f rad vel: %6.3f rad/s | " "[FB] pos: %6.3f rad vel: %6.3f rad/s\n", MITCtrlParam.pos, MITCtrlParam.vel, devicesState[IDswitch - 1].pos, devicesState[IDswitch - 1].vel );
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

  disable(0x01); // Disable motor with ID 0x01
  return 0;
}