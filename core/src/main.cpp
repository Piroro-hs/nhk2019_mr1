#include <mbed.h>

#include <BNO055.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>


#include "motor.h"

namespace {
  DigitalOut led(LED1);
  Motor shagai_pick_motor(PA_8, PC_11); // MD1
  Motor shagai_arm_motor(PA_0, PD_2); // MD2
  Motor gerege_arm_motor(PA_6, PC_9); // MD3
  Motor gerege_grab_motor(PB_6, PB_9); // MD4
  DigitalOut shagai_pick_solenoid(PB_15);
  DigitalOut shagai_throw_solenoid_on(PA_1);
  DigitalOut shagai_throw_solenoid_off(PB_14);
  // BNO055 imu(PB_7, PB_8); // RST PC8

// TIM3 下側flbr
  struct {
    bool enabled = true;
  } state = {};

  ros::NodeHandle nh;

  void onShagaiPickUp(const std_msgs::Bool& data) {
    shagai_pick_motor.drive(data.data);
  }

  void onShagaiPickDown(const std_msgs::Bool& data) {
    shagai_pick_motor.drive(static_cast<float>(data.data) * -0.5f);
  }

  void onShagaiArmExtend(const std_msgs::Bool& data) {
    shagai_arm_motor.drive(data.data);
  }

  void onShagaiArmCollapse(const std_msgs::Bool& data) {
    shagai_arm_motor.drive(static_cast<float>(data.data) * -1);
  }

  void onShagaiPick(const std_msgs::Bool& data) {
    shagai_pick_solenoid.write(data.data);
  }

  void onShagaiThrow(const std_msgs::Int8& data) {
    shagai_throw_solenoid_on.write(data.data == 1);
    shagai_throw_solenoid_off.write(data.data == -1);
  }

  void onGeregeArmMove(const std_msgs::Bool& data) {
    gerege_arm_motor.drive(static_cast<float>(data.data) * 0.5f);
  }

  void onGeregeGrab(const std_msgs::Int8& data) {
    gerege_grab_motor.drive(data.data);
  }

  ros::Subscriber<std_msgs::Bool> shagai_pick_up("shagai_pick_up", onShagaiPickUp);
  ros::Subscriber<std_msgs::Bool> shagai_pick_down("shagai_pick_down", onShagaiPickDown);
  ros::Subscriber<std_msgs::Bool> shagai_arm_extend("shagai_arm_extend", onShagaiArmExtend);
  ros::Subscriber<std_msgs::Bool> shagai_arm_collapse("shagai_arm_collapse", onShagaiArmCollapse);
  ros::Subscriber<std_msgs::Bool> shagai_pick("shagai_pick", onShagaiPick);
  ros::Subscriber<std_msgs::Int8> shagai_throw("shagai_throw", onShagaiThrow);
  ros::Subscriber<std_msgs::Bool> gerege_arm_move("gerege_arm_move", onGeregeArmMove);
  ros::Subscriber<std_msgs::Int8> gerege_grab("gerege_grab", onGeregeGrab);
}

int main() {
  nh.initNode();
  nh.subscribe(shagai_pick_up);
  nh.subscribe(shagai_pick_down);
  nh.subscribe(shagai_arm_extend);
  nh.subscribe(shagai_arm_collapse);
  nh.subscribe(shagai_pick);
  nh.subscribe(shagai_throw);
  nh.subscribe(gerege_arm_move);
  nh.subscribe(gerege_grab);

  // imu.reset();
  // imu.SetExternalCrystal(true);
  // imu.setmode(OPERATION_MODE_IMUPLUS);
  // wait(1);

  while (true) {
    nh.spinOnce();
    wait_ms(1);
  }
}
