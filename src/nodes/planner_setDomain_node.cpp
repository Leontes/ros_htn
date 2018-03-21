#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <fstream>


std::string domain, path;

void copyDomain(std::string dfile){
  domain = path + "/planningfiles/domain.pddl";
  std::ifstream inputDomain(dfile);
  std::ofstream outputDomain(domain);

  std::string line;
  while(!inputDomain.eof()){
    std::getline(inputDomain, line);
    outputDomain << line << std::endl;
  }

  inputDomain.close();
  outputDomain.close();
}

void updateDomainCB(const std_msgs::String::ConstPtr& msg){
  ROS_INFO_STREAM("New domain file: " + msg -> data);
  copyDomain(msg -> data);
}


int main(int argc, char *argv[]) {
  path = ros::package::getPath("ros_htn");

  if(argc > 1){
    copyDomain(std::string(argv[1]));
  }

  ros::init(argc, argv, "htn_domain_updater");
  ros::NodeHandle nhandler;
  ros::Subscriber sub = nhandler.subscribe("dom_updater", 1000, updateDomainCB);

  ros::spin();

  return 0;
}
