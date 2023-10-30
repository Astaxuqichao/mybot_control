#include <mecanumbot_hw/mecanumbot_hw_real.h>

namespace mybot_hw
{
  MecanumBOTHardwareInterface::MecanumBOTHardwareInterface(ros::NodeHandle &nh)
      : nh_(nh)
  {
    // Initialize shared memory and interfaces
    init();

    wheel_state_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>(
        "wheel_state_encoder", 1, &MecanumBOTHardwareInterface::wheelStateCB, this);
    wheel_cmd_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("wheel_cmd_velocity", 1);

    // Create the controller manager
    controller_manager_.reset(
        new controller_manager::ControllerManager(this, nh_));

    // Get period and create timer
    nh_.param("hardware_interface/loop_hz", loop_hz_, 10.);
    ROS_DEBUG_STREAM_NAMED("constructor",
                           "Using loop freqency of " << loop_hz_ << " hz");
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ =
        nh_.createTimer(update_freq, &MecanumBOTHardwareInterface::update, this);

    ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
  }

  MecanumBOTHardwareInterface::~MecanumBOTHardwareInterface() {}

  void MecanumBOTHardwareInterface::init()
  {

    // Get joint names
    nh_.getParam("hardware_interface/joints", joint_names_);
    if (joint_names_.size() == 0)
    {
      ROS_FATAL_STREAM_NAMED("init",
                             "Not joints found on parameter server for "
                             "controller, did you load the proper yaml file?");
    }
    num_joints_ = joint_names_.size();

    wheel_encoder_state_.data.resize(num_joints_);
    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);

    // Initialize controller
    for (int i = 0; i < num_joints_; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("constructor",
                             "Loading joint name: " << joint_names_[i]);

      // Create joint state interface
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i],
          &joint_effort_[i]));

      // Create position joint interface
      // position_joint_interface_.registerHandle(hardware_interface::JointHandle(
      //                                                                          joint_state_interface_.getHandle(joint_names_[i]),&joint_position_command_[i]));

      // Create velocity joint interface
      velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
          joint_state_interface_.getHandle(joint_names_[i]),
          &joint_velocity_command_[i]));

      // Create effort joint interface
      // effort_joint_interface_.registerHandle(hardware_interface::JointHandle(
      //    joint_state_interface_.getHandle(joint_names_[i]),&joint_effort_command_[i]));
    }
    registerInterface(&joint_state_interface_); // From RobotHW base class.
    // registerInterface(&position_joint_interface_); // From RobotHW base class.
    registerInterface(&velocity_joint_interface_); // From RobotHW base class.
    // registerInterface(&effort_joint_interface_);   // From RobotHW base class.
  }

  void MecanumBOTHardwareInterface::update(const ros::TimerEvent &e)
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);

    // Input
    read();

    // Control
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    write(elapsed_time_);
  }

  void MecanumBOTHardwareInterface::read()
  {
    if (wheel_encoder_state_.data.size() != num_joints_)
      return;

    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      // update position might add smoothing here later
      joint_position_[i] += wheel_encoder_state_.data[i];

    }
  }

  void MecanumBOTHardwareInterface::write(ros::Duration elapsed_time)
  {
    std_msgs::Float32MultiArray wheel_cmd;
    wheel_cmd.data.resize(num_joints_);
    // Send commands in different modes
    std::ostringstream os;
    // Move all the states to the commanded set points slowly
    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      wheel_cmd.data[i] = joint_velocity_command_[i];
      os << wheel_cmd.data[i] << "  ";
    }
    ROS_INFO_STREAM("Commands for joints: " << os.str());
    wheel_cmd_pub_.publish(wheel_cmd);
  }

  void MecanumBOTHardwareInterface::wheelStateCB(
      const std_msgs::Float32MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() != num_joints_)
    {
      ROS_INFO_STREAM("hardwarw input error");
      return;
    }
    wheel_encoder_state_ = *msg;
  }

} // namespace mybot_hw
