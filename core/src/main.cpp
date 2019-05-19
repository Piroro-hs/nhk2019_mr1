#include <mbed.h>

#include <BNO055.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

#include "motor.h"
#include "quat.h"

namespace {
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

  void onShagaiPick(const std_msgs::Int8& data) {
    shagai_pick_motor.drive(data.data * -0.5f);
  }

  void onShagaiArm(const std_msgs::Int8& data) {
    shagai_arm_motor.drive(data.data * -1);
  }

  void onShagaiGrab(const std_msgs::Bool& data) {
    shagai_grab_solenoid.write(data.data);
  }

  void onShagaiThrow(const std_msgs::Int8& data) {
    shagai_throw_solenoid_on.write(data.data == 1);
    shagai_throw_solenoid_off.write(data.data == -1);
  }

  void onGeregeActuator(const std_msgs::UInt8& data) {
    const auto arm = data.data & 0b0001 ? 1 : data.data & 0b0010 ? -1 : 0;
    const auto grab = data.data & 0b0100 ? 1 : data.data & 0b1000 ? -1 : 0;
    gerege_arm_motor.drive(arm * 0.5f);
    gerege_grab_motor.drive(grab);
  }

  ros::Subscriber<std_msgs::Int8> shagai_pick("shagai_pick", onShagaiPick);
  ros::Subscriber<std_msgs::Int8> shagai_arm("shagai_arm", onShagaiArm);
  ros::Subscriber<std_msgs::Bool> shagai_grab("shagai_grab", onShagaiGrab);
  ros::Subscriber<std_msgs::Int8> shagai_throw("shagai_throw", onShagaiThrow);
  ros::Subscriber<std_msgs::UInt8> gerege_actuator("gerege_actuator", onGeregeActuator);
}

int main() {
  nodeHandle.getHardware()->setBaud(2000000);
  nodeHandle.initNode();
  nodeHandle.subscribe(shagai_pick);
  nodeHandle.subscribe(shagai_arm);
  nodeHandle.subscribe(shagai_grab);
  nodeHandle.subscribe(shagai_throw);
  nodeHandle.subscribe(gerege_actuator);
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
