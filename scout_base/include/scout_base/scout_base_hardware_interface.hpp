#ifndef SCOUT_BASE__SCOUT_BASE_HARDWARE_INTERFACE_HPP_
#define SCOUT_BASE__SCOUT_BASE_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "scout_base/scout_messenger.hpp"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scout_base_hardware_interface
{
class ScoutBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init( const hardware_interface::HardwareComponentInterfaceParams &hardware_info ) override;

  hardware_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State &previous_state ) override;

  hardware_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State &previous_state ) override;

  hardware_interface::CallbackReturn
  on_deactivate( const rclcpp_lifecycle::State &previous_state ) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  hardware_interface::return_type read( const rclcpp::Time &time,
                                        const rclcpp::Duration &period ) override;

  hardware_interface::return_type write( const rclcpp::Time &time,
                                         const rclcpp::Duration &period ) override;

private:
  template<typename T>
  bool getParameter( const std::unordered_map<std::string, std::string> &map,
                     const std::string &param_name, T &value );

  bool SetUpMessenger();

  struct InterfaceData {
    explicit InterfaceData( const std::string &name );

    std::string name_;
    double command_;
    double state_;
  };

  double linear_goal_vel_;
  double angular_goal_vel_;

  std::vector<InterfaceData> joint_interfaces_;
  // std::vector<InterfaceData> actuator_interfaces_;

  // std::shared_ptr<rclcpp::Node> scout_status_node_;
  // std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe_;
  // std::thread status_pub_thread_;

  // instantiate a robot object
  std::shared_ptr<westonrobot::ScoutRobot> scout_interface_;
  std::unique_ptr<westonrobot::ScoutMessenger<westonrobot::ScoutRobot>> messenger_;

  std::string port_name_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool is_scout_mini_ = false;
  bool auto_reconnect_ = true;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double map_position_x_;
  double map_position_y_;
  double map_position_z_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace scout_base_hardware_interface

#endif // SCOUT_BASE__SCOUT_BASE_HARDWARE_INTERFACE_HPP_