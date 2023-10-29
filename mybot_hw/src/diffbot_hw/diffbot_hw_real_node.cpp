#include <diffbot_hw/diffbot_hw_real.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "diffbot_hw_real");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(4);
  spinner.start();

  mybot_hw::DiffBOTHardwareInterface diff_bot(nh);

  ros::waitForShutdown();

  return 0;
}
