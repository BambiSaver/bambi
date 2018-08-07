#include <ros/ros.h>
#include "lib/statemachine.h"
#include "lib/mcpublisher.h"

#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/BambiMissionTrigger.h>

using namespace bambi::missioncontroller;

StateMachine* stateMachinePtr = NULL;


void cb_missionTriggerReceived(const mavros_msgs::BambiMissionTrigger &msg) {
  stateMachinePtr->missionTriggerReceived(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bambi");
  ros::NodeHandle nh("~missioncontroller");
  
  MCPublisher mcpublisher(nh);
  StateMachine stateMachine(mcpublisher);
  
  stateMachinePtr = &stateMachine;
  
  nh.subscribe("/mavros/bambi/missiontrigger", 1000,
               cb_missionTriggerReceived);
  
  
  ROS_INFO("Mission Controller STARTUP");


  while(ros::ok()) {
    ros::spinOnce();
  }
}
