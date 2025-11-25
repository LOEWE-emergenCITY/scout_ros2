#include <hardware_interface/system_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_base/scout_base_hardware_interface.hpp"

namespace scout_base_hardware_interface
{

template<typename T>
T parse( std::string value ) {};

template<>
bool parse( std::string value )
{
  return hardware_interface::parse_bool( value );
}

template<>
double parse( std::string value )
{
  return hardware_interface::stod( value );
}

template<>
std::string parse( std::string value )
{
  return value;
}

template<typename T>
bool ScoutBaseHardwareInterface::getParameter( const std::unordered_map<std::string, std::string> &map,
                                               const std::string &param_name, T &value )
{
  try {
    value = parse<T>( map.at( param_name ) );
  } catch ( const std::out_of_range & ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Parameter %s not found", param_name.c_str() );
    return false;
  } catch ( std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error while loading parameter %s: %s",
                  param_name.c_str(), e.what() );
    return false;
  }
  return true;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ScoutBaseHardwareInterface::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.emplace_back( std::make_shared<hardware_interface::CommandInterface>(
      "scout_base", "linear_vel", &linear_goal_vel_ ) );
  command_interfaces.emplace_back( std::make_shared<hardware_interface::CommandInterface>(
      "scout_base", "angular_vel", &angular_goal_vel_ ) );

  return command_interfaces;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ScoutBaseHardwareInterface::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces =
      std::vector<hardware_interface::StateInterface::ConstSharedPtr>{ {
          std::make_shared<hardware_interface::StateInterface>( "scout_base", "map_position_x",
                                                                &map_position_x_ ),
          std::make_shared<hardware_interface::StateInterface>( "scout_base", "map_position_y",
                                                                &map_position_y_ ),
          std::make_shared<hardware_interface::StateInterface>( "scout_base", "map_position_z",
                                                                &map_position_z_ ),
      } };

  return state_interfaces;
}

hardware_interface::CallbackReturn
ScoutBaseHardwareInterface::on_init( const hardware_interface::HardwareComponentInterfaceParams & hardware_info )
{
  // Load hardware configuration
  const auto result = SystemInterface::on_init( hardware_info );
  if ( result != CallbackReturn::SUCCESS ) {
    return result;
  }

  // Load parameters
  const auto &info_ = hardware_info.hardware_info;
  if ( !getParameter<std::string>( info_.hardware_parameters, "port_name", port_name_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  if ( !getParameter<bool>( info_.hardware_parameters, "is_scout_mini", is_scout_mini_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  if ( !getParameter<std::string>( info_.hardware_parameters, "odom_topic", odom_topic_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  if ( !getParameter<std::string>( info_.hardware_parameters, "odom_frame", odom_frame_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  if ( !getParameter<std::string>( info_.hardware_parameters, "base_frame", base_frame_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  if ( !getParameter<bool>( info_.hardware_parameters, "auto_reconnect", auto_reconnect_ ) )
    return hardware_interface::CallbackReturn::ERROR;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ScoutBaseHardwareInterface::on_configure( const rclcpp_lifecycle::State & )
{
  // Messenger also handles connection to robot via CAN interface
  if ( !SetUpMessenger() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Failed to set up messenger for scout hardware interface" );
    return hardware_interface::CallbackReturn::ERROR;
  }

  // auto ns = std::string( get_node->get_namespace() );
  // scout_status_node_ = std::make_shared<rclcpp::Node>(
  //     hardware_info.name, ns, rclcpp::NodeOptions().use_global_arguments( false ) );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_node()->get_clock() );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

  // exe_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // exe_->add_node( scout_status_node_ );

  // setUpStatusPubThread();

  status_timer_ = get_node()->create_wall_timer( std::chrono::milliseconds( 50 ), [this]() {
    messenger_->PublishStateToROS();
    AgxControlMode robot_control;
    robot_control = scout_interface_->GetRobotState().system_state.control_mode;
    if ( auto_reconnect_ && robot_control == CONTROL_MODE_STANDBY ) {
      scout_interface_->EnableCommandedMode();
    }
  } );

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool ScoutBaseHardwareInterface::SetUpMessenger()
{
  bool success = westonrobot::ScoutMessenger<westonrobot::ScoutRobot>::CreateRobotInterface(
      port_name_, is_scout_mini_, scout_interface_, get_node()->get_logger() );

  if ( !success ) {
    return false;
  }

  messenger_ = std::unique_ptr<westonrobot::ScoutMessenger<westonrobot::ScoutRobot>>(
      new westonrobot::ScoutMessenger<westonrobot::ScoutRobot>( scout_interface_, get_node() ) );

  messenger_->SetOdometryFrame( odom_frame_ );
  messenger_->SetBaseFrame( base_frame_ );
  messenger_->SetOdometryTopicName( odom_topic_ );
  messenger_->SetEnableCmdSubs( false );

  messenger_->SetupSubscription();

  return true;
}

/*void ScoutBaseHardwareInterface::StatusTimerCb()
{
  exe_thread_ = std::thread( [this] {
    AgxControlMode robot_control;
    rclcpp::Rate rate( 50 );
    while ( rclcpp::ok() ) {
      messenger_->PublishStateToROS();
      robot_control = scout_interface_->GetRobotState().system_state.control_mode;
      if ( auto_reconnect_ && robot_control == CONTROL_MODE_STANDBY ) {
        scout_interface_->EnableCommandedMode();
      }
      exe_->spin_some();
      rate.sleep();
    }
  } );
}*/

/*ScoutBaseHardwareInterface::~ScoutBaseHardwareInterface()
{
  try {
    if ( exe_ ) {
      exe_->cancel();
      if ( scout_status_node_ ) {
        try {
          exe_->remove_node( scout_status_node_ );
        } catch ( ... ) {
        }
      }
    }
    if ( exe_thread_
.joinable() ) {
      exe_thread_.join();
    }
  } catch ( ... ) {
  }
  exe_.reset();
  scout_status_node_.reset();
}*/

hardware_interface::CallbackReturn
ScoutBaseHardwareInterface::on_activate( const rclcpp_lifecycle::State & )
{
  linear_goal_vel_ = 0.0;
  angular_goal_vel_ = 0.0;

  status_timer_->reset();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ScoutBaseHardwareInterface::on_deactivate( const rclcpp_lifecycle::State & )
{
  linear_goal_vel_ = 0.0;
  angular_goal_vel_ = 0.0;

  status_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ScoutBaseHardwareInterface::read( const rclcpp::Time & time,
                                                                  const rclcpp::Duration &)
{
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer_->lookupTransform( "map", base_frame_, time );
  } catch ( const tf2::TransformException &ex ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Could not transform %s to %s: %s", "map",
                  base_frame_.c_str(), ex.what() );
    return hardware_interface::return_type::ERROR;
  }

  map_position_x_ = t.transform.translation.x;
  map_position_y_ = t.transform.translation.y;
  map_position_z_ = t.transform.translation.z;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ScoutBaseHardwareInterface::write( const rclcpp::Time &,
                                                                   const rclcpp::Duration & )
{
  try {
    scout_interface_->SetMotionCommand( linear_goal_vel_, angular_goal_vel_ );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception in write(): %s", e.what() );
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace scout_base_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(scout_base_hardware_interface::ScoutBaseHardwareInterface, hardware_interface::SystemInterface)