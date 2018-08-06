#include <ros/ros.h>
//#include <ostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "missioncontroller");
  ros::NodeHandle nh("/bambi/missioncontroller");

  ROS_INFO("Hello world! I'm the missioncontroller");

  while(ros::ok()) {
    ros::spinOnce();
    //std::cout
  }
}
