#include <ros/ros.h>
#include <nmcp_navigation.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_mpc_navigation");
  ros::NodeHandle nh;

  NMPCControllerROS nmpc_ros;
  nmpc_ros.run();
  
  return 0;
}
