#pragma once

#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <mecanum_drive_controller/MecanumDriveControllerConfig.h>
#include <mecanum_drive_controller/odometry.h>
#include <mecanum_drive_controller/speed_limiter.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>

namespace mecanum_drive_controller {

class MecanumDriveController : public controller_interface::Controller<
                                   hardware_interface::VelocityJointInterface> {
public:
  MecanumDriveController();

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface *hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new
   * velocity commands \param time   Current time \param period Time since the
   * last called to update
   */
  void update(const ros::Time &time, const ros::Duration &period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  void starting(const ros::Time &time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  void stopping(const ros::Time & /*time*/);

private:
  std::string name_;

  /// Odometry related:
  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  bool open_loop_;

  /// Hardware handles:
  std::vector<hardware_interface::JointHandle> left_wheel_joints_;
  std::vector<hardware_interface::JointHandle> right_wheel_joints_;

  // Previous time
  ros::Time time_previous_;

  /// Previous velocities from the encoders:
  std::vector<double> vel_left_previous_;
  std::vector<double> vel_right_previous_;

  /// Previous velocities from the encoders:
  std::vector<double> vel_left_desired_previous_;
  std::vector<double> vel_right_desired_previous_;

  /// Velocity command related:
  struct Commands {
    double lin_x;
    double lin_y;
    double ang;
    ros::Time stamp;

    Commands() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;

  /// Publish executed commands
  std::shared_ptr<
      realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>>
      cmd_vel_pub_;

  /// Odometry related:
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
      odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>>
      tf_odom_pub_;
  Odometry odometry_;

  /// Controller state publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<
      control_msgs::JointTrajectoryControllerState>>
      controller_state_pub_;

  /// Wheel separation, wrt the midpoint of the wheel width:
  double wheel_separation_;
  /// distance of front wheel and rear wheel
  double wheel_distance_;

  /// Wheel radius (assuming it's the same for the left and right wheels):
  double wheel_radius_;

  /// Wheel separation and radius calibration multipliers:
  double wheel_separation_multiplier_;
  double wheel_distance_multiplier_;
  double left_front_wheel_radius_multiplier_;
  double right_front_wheel_radius_multiplier_;
  double left_rear_wheel_radius_multiplier_;
  double right_rear_wheel_radius_multiplier_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Whether to allow multiple publishers on cmd_vel topic or not:
  bool allow_multiple_cmd_vel_publishers_;

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Frame to use for odometry and odom tf:
  std::string odom_frame_id_;

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;

  /// Number of wheel joints:
  size_t wheel_joints_size_;

  /// Speed limiters:
  Commands last1_cmd_;
  Commands last0_cmd_;
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

  /// Publish limited velocity:
  bool publish_cmd_;

  /// Publish wheel data:
  bool publish_wheel_joint_controller_state_;

  // A struct to hold dynamic parameters
  // set from dynamic_reconfigure server
  struct DynamicParams {
    bool update;

    double left_front_wheel_radius_multiplier;
    double right_front_wheel_radius_multiplier;
    double left_rear_wheel_radius_multiplier;
    double right_rear_wheel_radius_multiplier;
    double wheel_separation_multiplier;
    double wheel_distance_multiplier;

    bool publish_cmd;

    double publish_rate;
    bool enable_odom_tf;

    DynamicParams()
        : left_front_wheel_radius_multiplier(1.0),
          right_front_wheel_radius_multiplier(1.0),
          left_rear_wheel_radius_multiplier(1.0),
          right_rear_wheel_radius_multiplier(1.0),
          wheel_separation_multiplier(1.0), publish_cmd(false),
          publish_rate(50), enable_odom_tf(true) {}

    friend std::ostream &operator<<(std::ostream &os,
                                    const DynamicParams &params) {
      os << "DynamicParams:\n"
         //
         << "\tOdometry parameters:\n"
         << "\t\tleft wheel radius multiplier: "
         << params.left_front_wheel_radius_multiplier << "\n"
         << "\t\tright wheel radius multiplier: "
         << params.right_front_wheel_radius_multiplier << "\n"
         << "\t\tleft wheel radius multiplier: "
         << params.left_rear_wheel_radius_multiplier << "\n"
         << "\t\tright wheel radius multiplier: "
         << params.right_rear_wheel_radius_multiplier << "\n"
         << "\t\twheel separation multiplier: "
         << params.wheel_separation_multiplier
         << "\n"
         //
         << "\tPublication parameters:\n"
         << "\t\tPublish executed velocity command: "
         << (params.publish_cmd ? "enabled" : "disabled") << "\n"
         << "\t\tPublication rate: " << params.publish_rate << "\n"
         << "\t\tPublish frame odom on tf: "
         << (params.enable_odom_tf ? "enabled" : "disabled");

      return os;
    }
  };

  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;

  /// Dynamic Reconfigure server
  typedef dynamic_reconfigure::Server<MecanumDriveControllerConfig>
      ReconfigureServer;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
  boost::recursive_mutex dyn_reconf_server_mutex_;

private:
  /**
   * \brief Brakes the wheels, i.e. sets the velocity to 0
   */
  void brake();

  /**
   * \brief Velocity command callback
   * \param command Velocity command message (twist)
   */
  void cmdVelCallback(const geometry_msgs::Twist &command);

  /**
   * \brief Get the wheel names from a wheel param
   * \param [in]  controller_nh Controller node handler
   * \param [in]  wheel_param   Param name
   * \param [out] wheel_names   Vector with the whel names
   * \return true if the wheel_param is available and the wheel_names are
   *        retrieved successfully from the param server; false otherwise
   */
  bool getWheelNames(ros::NodeHandle &controller_nh,
                     const std::string &wheel_param,
                     std::vector<std::string> &wheel_names);

  /**
   * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and
   * separation \param root_nh Root node handle \param left_wheel_name Name of
   * the left wheel joint \param right_wheel_name Name of the right wheel joint
   */
  bool setOdomParamsFromUrdf(ros::NodeHandle &root_nh,
                             const std::string &left_front_wheel_name,
                             const std::string &left_rear_wheel_name,
                             const std::string &right_front_wheel_name,
                             const std::string &right_rear_wheel_name,
                             bool lookup_wheel_separation,
                             bool lookup_wheel_distance,
                             bool lookup_wheel_radius);

  /**
   * \brief Sets the odometry publishing fields
   * \param root_nh Root node handle
   * \param controller_nh Node handle inside the controller namespace
   */
  void setOdomPubFields(ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh);

  /**
   * \brief Callback for dynamic_reconfigure server
   * \param config The config set from dynamic_reconfigure server
   * \param level not used at this time.
   * \see dyn_reconf_server_
   */
  void reconfCallback(MecanumDriveControllerConfig &config, uint32_t /*level*/);

  /**
   * \brief Update the dynamic parameters in the RT loop
   */
  void updateDynamicParams();

  /**
   * \brief
   * \param time Current time
   * \param period Time since the last called to update
   * \param curr_cmd Current velocity command
   * \param wheel_separation wheel separation with multiplier
   * \param left_wheel_radius left wheel radius with multiplier
   * \param right_wheel_radius right wheel radius with multiplier
   */
  void publishWheelData(const ros::Time &time, const ros::Duration &period,
                        const std::vector<double> vel_left_desired,
                        const std::vector<double> right_left_desired);
};

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController,
                       controller_interface::ControllerBase);
} // namespace mecanum_drive_controller
