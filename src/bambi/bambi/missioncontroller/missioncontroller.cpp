#include <ros/ros.h>
#include "lib/statemachine.h"
#include "lib/mcpublisher.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/BambiMissionTrigger.h>

using namespace bambi::missioncontroller;

StateMachine* stateMachinePtr = NULL;

void cb_missionTriggerReceived(const mavros_msgs::BambiMissionTrigger &msg) {
  stateMachinePtr->missionTriggerReceived(msg);
}
void cb_uavStateChange(const mavros_msgs::State &msg) {
  stateMachinePtr->uavStateChange(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bambi");
  ros::NodeHandle nh("~missioncontroller");
  
  MCPublisher mcpublisher(nh);
  StateMachine stateMachine(mcpublisher);
  
  stateMachinePtr = &stateMachine;
  
  ros::Subscriber s1 = nh.subscribe("/mavros/bambi/missiontrigger", 10,
               cb_missionTriggerReceived);
  ros::Subscriber s2 = nh.subscribe("/mavros/state", 10,
               cb_uavStateChange);
  
  
  //ROS_INFO("Subscriber topic: %s, count: %d", s1.getTopic().c_str(), s1.getNumPublishers());
  //ROS_INFO("Subscriber topic: %s, count: %d", s2.getTopic().c_str(), s2.getNumPublishers());
  
  ROS_INFO("Mission Controller STARTUP");

  while(ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
