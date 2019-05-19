#include <mbed.h>

#include <BNO055.h>
#include <rotary_encoder_ab_phase.hpp>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>

#include "motor.h"
#include "quat.h"

namespace {
  constexpr size_t ODOM_PUB_RATE = 10;
  ros::Time prevOdomPubTime;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub("odom", &odom_msg);

  Quat prev_q;

  DigitalOut led(LED1);
  Motor shagai_pick_motor(PA_8, PC_11); // MD1
  Motor shagai_arm_motor(PA_0, PD_2); // MD2
  Motor gerege_arm_motor(PA_6, PC_9); // MD3
  Motor gerege_grab_motor(PB_6, PB_9); // MD4
  DigitalOut shagai_grab_solenoid(PB_15);
  DigitalOut shagai_throw_solenoid_on(PA_1);
  DigitalOut shagai_throw_solenoid_off(PB_14);
  BNO055 imu(PB_7, PB_8); // RST PC8
  // rotary_encoder_ab_phase frbl_enc(TIM3, 500);
  // rotary_encoder_ab_phase flbr_enc(TIM8, 500);

  struct {
    bool enabled = true;
  } state = {};

  // ros::NodeHandle nh;
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
  rotary_encoder_ab_phase frbl_enc(TIM3, 500);
  rotary_encoder_ab_phase flbr_enc(TIM8, 500);

  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  nodeHandle.getHardware()->setBaud(2000000);
  nodeHandle.initNode();
  nodeHandle.subscribe(shagai_pick);
  nodeHandle.subscribe(shagai_arm);
  nodeHandle.subscribe(shagai_grab);
  nodeHandle.subscribe(shagai_throw);
  nodeHandle.subscribe(gerege_actuator);
  nodeHandle.advertise(odom_pub);

  // imu_msg.header.frame_id = "map";
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  imu.reset();
  imu.SetExternalCrystal(true);
  // imu.set_mapping(2);
  imu.setmode(OPERATION_MODE_IMUPLUS);
  wait(1);

  flbr_enc.start();
  frbl_enc.start();

  while (!nodeHandle.connected()){
    nodeHandle.spinOnce();
    wait_ms(10);
  }

  auto curTime = nodeHandle.now();
  prevOdomPubTime = curTime;
  imu.get_quat();
  prev_q = Quat(imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);

  while (true) {
    curTime = nodeHandle.now();
    if (auto duration = curTime.nsec - prevOdomPubTime.nsec; duration > 1000000000 / ODOM_PUB_RATE) {
      prevOdomPubTime += ros::Duration(0, 1000000000 / ODOM_PUB_RATE);
      constexpr auto SQRT_2 = 1.41421356f;
      constexpr auto DIV_BY_SQRT_2 = 1 / SQRT_2;
      constexpr auto r = 0.03f;
      constexpr auto l = 0.08925f;
      const auto omega1 = flbr_enc.get_revol_num() * 1000000000 / duration;
      const auto omega2 = frbl_enc.get_revol_num() * 1000000000 / duration;
      // flbr_enc.reset();
      // frbl_enc.reset();
      char buffer[32];
      snprintf(buffer, sizeof buffer, "omega1: %d", flbr_enc.get_counts());
      nodeHandle.loginfo(buffer);
      char buffer2[32];
      snprintf(buffer2, sizeof buffer2, "omega2: %d", frbl_enc.get_counts());
      nodeHandle.loginfo(buffer2);
      imu.get_quat();
      const Quat cur_q(imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);
      auto yaw = cur_q.product(prev_q.conjugate()).getYaw();
      const auto vx = -DIV_BY_SQRT_2 * r * (omega1 - omega2);
      const auto vy = -DIV_BY_SQRT_2 * r * (omega1 - omega2) - SQRT_2 * l * yaw;
      prev_q = cur_q;
      odom_msg.pose.pose.position.x += vx * duration / 1000000000;
      odom_msg.pose.pose.position.y += vy * duration / 1000000000;
      odom_msg.pose.pose.orientation.x = imu.quat.x;
      odom_msg.pose.pose.orientation.y = imu.quat.y;
      odom_msg.pose.pose.orientation.z = imu.quat.z;
      odom_msg.pose.pose.orientation.w = imu.quat.w;
      odom_msg.twist.twist.linear.x = vx;
      odom_msg.twist.twist.linear.y = vy;
      odom_msg.twist.twist.angular.z = yaw;
      odom_msg.header.stamp = curTime;
      odom_pub.publish(&odom_msg);
    }
    nodeHandle.spinOnce();
    wait_ms(10);
  }
}
