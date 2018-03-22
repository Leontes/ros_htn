#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <fstream>

#include "executecommand.hpp"

std::string path;

void updateDomainCB(const std_msgs::String::ConstPtr& msg){
  ROS_INFO_STREAM("New domain file: " + msg -> data);
  exec("cp " + msg -> data + " " + path + "/planningfiles/domain.pddl");
}


int main(int argc, char *argv[]) {
  path = ros::package::getPath("ros_htn");

  std::string inDom;

  ros::init(argc, argv, "htn_domain_updater");
  ros::NodeHandle nhandler;

  nhandler.getParam("/domain_updater_node/inParam", inDom);

  if( inDom != (path + "/planningfiles/domain.hpp")){
    exec("cp " + inDom + " " + path + "/planningfiles/domain.hpp");
  }

  ros::Subscriber sub = nhandler.subscribe("dom_updater", 1000, updateDomainCB);

  ros::spin();

  return 0;
}
