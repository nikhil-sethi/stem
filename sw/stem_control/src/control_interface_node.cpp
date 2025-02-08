#include "stem_control/fuel_interface.h"


int main(int argc, char **argv)
{
  try {
    // Initialize ROS node
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");

    // Create PX4 control interface instance
    Controller* controller = new FUELInterface(nh);

    // Initialize controller and catch possible errors
    if (!controller->initialize()) {
      CONTROLLER_ERROR("Controller initialization failed!");
      exit(1);
    } else {
      CONTROLLER_INFO_ALWAYS("Controller initialization successful");
      ros::spin();
    }
  } catch (ros::Exception& e) {
    CONTROLLER_ERROR_STREAM("Error occurred: " << e.what() << "!");
    exit(1);
  }

  return 0;
}
