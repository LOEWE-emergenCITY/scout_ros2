/*
 * scout_base_node.cpp
 *
 * Created on: Oct 15, 2021 16:20
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_base/scout_base_ros.hpp"

using namespace westonrobot;

std::shared_ptr<ScoutBaseRos> robot;

void DetachRobot(int signal) {
  (void)signal;
  robot->Stop();
}

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  //   std::signal(SIGINT, DetachRobot);

  while (rclcpp::ok()) {
    try {
      robot = std::make_shared<ScoutBaseRos>("scout");
      if (robot->Initialize()) {
        std::cout << "Robot initialized, start running ..." << std::endl;
        robot->Run();
      }
      robot->Stop();
      return 0;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("scout_base_node"), "Exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("scout_base_node"), "Unknown exception");
    }
    RCLCPP_INFO(rclcpp::get_logger("scout_base_node"), "Retrying...");
  }

  return 0;
}