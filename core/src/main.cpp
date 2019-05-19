#include <mbed.h>

#include <BNO055.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

#include "motor.h"
#include "quat.h"

namespace {
  constexpr uint16_t MASK_SP_F = 0x01 << 0;
  constexpr uint16_t MASK_SP_B = 0x01 << 1;
  constexpr uint16_t MASK_SA_F = 0x01 << 2;
  constexpr uint16_t MASK_SA_B = 0x01 << 3;
  constexpr uint16_t MASK_SG = 0x01 << 4;
  constexpr uint16_t MASK_ST_F = 0x01 << 5;
  constexpr uint16_t MASK_ST_B = 0x01 << 6;
  constexpr uint16_t MASK_GA_F = 0x01 << 7;
  constexpr uint16_t MASK_GA_B = 0x01 << 8;
  constexpr uint16_t MASK_GG_F = 0x01 << 9;
  constexpr uint16_t MASK_GG_B = 0x01 << 10;

  constexpr size_t ANGLE_PUB_RATE = 60;
  ros::Time prevAnglePubTime;
  std_msgs::Float32 angle_msg;
  ros::Publisher angle_pub("angle", &angle_msg);

  DigitalOut led(LED1);
  Motor shagai_pick_motor(PA_8, PC_11); // MD1
  Motor shagai_arm_motor(PA_0, PD_2); // MD2
  Motor gerege_arm_motor(PA_6, PC_9); // MD3
  Motor gerege_grab_motor(PB_6, PB_9); // MD4
  DigitalOut shagai_grab_solenoid(PB_15);
  DigitalOut shagai_throw_solenoid_on(PA_1);
  DigitalOut shagai_throw_solenoid_off(PB_14);
  BNO055 imu(PB_7, PB_8); // RST PC8

// TIM3 下側flbr
  struct {
    bool enabled = true;
  } state = {};

  ros::NodeHandle_<MbedHardware, 25, 25, 2048, 16384> nodeHandle;

  void onActuatorStateChange(const std_msgs::UInt16& data) {
    const auto sp = data.data & MASK_SP_F ? 1 : data.data & MASK_SP_B ? -1 : 0;
    const auto sa = data.data & MASK_SA_F ? 1 : data.data & MASK_SA_B ? -1 : 0;
    const auto sg = data.data & MASK_SG;
    const auto st = data.data & MASK_ST_F ? 1 : data.data & MASK_ST_B ? -1 : 0;
    const auto ga = data.data & MASK_GA_F ? 1 : data.data & MASK_GA_B ? -1 : 0;
    const auto gg = data.data & MASK_GG_F ? 1 : data.data & MASK_GG_B ? -1 : 0;
    shagai_pick_motor.drive(sp * -0.5f);
    shagai_arm_motor.drive(sa * -1);
    shagai_grab_solenoid.write(sg);
    shagai_throw_solenoid_on.write(st == 1);
    shagai_throw_solenoid_off.write(st == -1);
    gerege_arm_motor.drive(ga * 0.5f);
    gerege_grab_motor.drive(gg);
  }

  ros::Subscriber<std_msgs::UInt16> actuator("actuator", onActuatorStateChange);
}

int main() {
  nodeHandle.getHardware()->setBaud(2000000);
  nodeHandle.initNode();
  nodeHandle.subscribe(actuator);
  nodeHandle.advertise(angle_pub);

  imu.reset();
  imu.SetExternalCrystal(true);
  // imu.set_mapping(2);
  imu.setmode(OPERATION_MODE_IMUPLUS);
  wait(1);

  while (!nodeHandle.connected()){
    nodeHandle.spinOnce();
    wait_ms(10);
  }

  auto curTime = nodeHandle.now();
  prevAnglePubTime = curTime;

  while (true) {
    curTime = nodeHandle.now();
    if (auto duration = curTime.nsec - prevAnglePubTime.nsec; duration > 1000000000 / ANGLE_PUB_RATE) {
      prevAnglePubTime += ros::Duration(0, 1000000000 / ANGLE_PUB_RATE);
      imu.get_quat();
      const Quat q(imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);
      angle_msg.data = q.getYaw();
      angle_pub.publish(&angle_msg);
    }
    nodeHandle.spinOnce();
    wait_ms(10);
  }
}
