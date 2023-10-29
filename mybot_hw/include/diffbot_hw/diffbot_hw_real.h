#ifndef MYBOT_CONTROL__DIFFBOT_HARDWARE_INTERFACE_REAL_H
#define MYBOT_CONTROL__DIFFBOT_HARDWARE_INTERFACE_REAL_H

#include <controller_manager/controller_manager.h>
#include <std_msgs/Float32MultiArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
// RRBOT hardware base class
#include <mybot_hw/mybot_hw.h>

namespace mybot_hw {

/// \brief Hardware interface for a robot
class DiffBOTHardwareInterface : public mybot_hw::mybotHardware {
public:
  /// \brief Constructor.
  ///
  /// \param nh  Node handle for topics.
  DiffBOTHardwareInterface(ros::NodeHandle &nh);

  /// \brief Destructor.
  ~DiffBOTHardwareInterface();

  /// \brief Initialize the hardware interface
  void init();

  /// \brief Timer event
  void update(const ros::TimerEvent &e);

  /// \brief Read the state from the robot hardware.
  void read();

  /// \brief write the command to the robot hardware.
  void write(ros::Duration elapsed_time);

  ros::Time getTime() const { return ros::Time::now(); }

  void wheelStateCB(const std_msgs::Float32MultiArray::ConstPtr &msg);

protected:
  ros::NodeHandle nh_;

  // Timing
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

  ros::Subscriber wheel_state_sub_;
  ros::Publisher wheel_cmd_pub_;
  std_msgs::Float32MultiArray wheel_encoder_state_;

}; // class

} // namespace mybot_hw

#endif
