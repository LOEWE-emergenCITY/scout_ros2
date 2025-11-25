#ifndef SCOUT_BASE_STATUS_PUB_HPP
#define SCOUT_BASE_STATUS_PUB_HPP

#include <rclcpp/rclcpp.hpp>

// Sub class of ScoutBaseRos to publish Scout status messages
namespace westonrobot
{

class ScoutBaseStatusPub : public rclcpp::Node
{
public:
  ScoutBaseStatusPub( std::string node_name, std::shared_ptr<ScoutRobot> robot, std::string port_name,
                      std::string odom_frame, std::string base_frame, std::string odom_topic_name,
                      bool is_scout_mini, bool is_omni_wheel, bool auto_reconnect )
      : rclcpp::Node( node_name ), robot_( robot ), port_name_( port_name ),
        odom_frame_( odom_frame ), base_frame_( base_frame ), odom_topic_name_( odom_topic_name ),
        is_scout_mini_( is_scout_mini ), auto_reconnect_( auto_reconnect ){};

  void Run();
  void Stop();

private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_scout_mini_ = false;
  bool auto_reconnect_;

  std::shared_ptr<ScoutRobot> robot_;
  std::atomic<bool> keep_running_;
}