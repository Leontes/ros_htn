#include <ros/ros.h>
#include <ros/package.h>
#include <ros_htn/GetPlan.h>
#include <ros_htn/PlanningProblem.h>

#include <vector>
#include <fstream>

#include "executecommand.hpp"

std::string domain, path;

std::string delimiter = ":action ";


void saveProblem(ros_htn::PlanningProblem problem){

  std::string domainName, line;
  std::size_t found = std::string::npos, dName;

  std::ifstream dfile(domain);
  while(found == std::string::npos){
    std::getline(dfile, line);
    found = line.find("domain");
    if (found!=std::string::npos){
      found = line.find(" ", found);
      dName = line.find(")", found+1);
      domainName = line.substr(found+1, dName-(found+1));
    }
  }
  dfile.close();


  std::ofstream planFile (path + "/planningfiles/lastPlan.pddl");

  planFile << "(define (problem lastPlan) (:domain " + domainName + ")" << std::endl;

  planFile << "\t(:objects" << std::endl;
  for(size_t i = 0; i < problem.objects.size(); i++){
    planFile << "\t\t" << (problem.objects[i]).objName << " - " + (problem.objects[i]).objType << std::endl;
  }
  planFile << "\t)" << std::endl;

  planFile << "\t(:init" << std::endl;
  for(size_t i = 0; i < problem.initState.size(); i++){
    planFile << "\t\t" << problem.initState[i] << std::endl;
  }
  planFile << "\t)" << std::endl;

  planFile << "\t(:tasks-goal\n\t\t:tasks(" << std::endl;
  for(size_t i = 0; i < problem.goal.size(); i++){
    planFile << "\t\t\t" << problem.goal[i] << std::endl;
  }
  planFile << "\t\t)\n\t)\n)" << std::endl;

  planFile.close();
}



bool plan(ros_htn::GetPlan::Request &req, ros_htn::GetPlan::Response &resp){
  saveProblem(req.problem);

  std::string outPlan,token;
  ROS_INFO_STREAM(path + std::string("/htnplanner/planner -v2 -d ") + domain + std::string(" -p ") + path + std::string("/planningfiles/lastPlan.pddl"));
  outPlan = exec(path + std::string("/htnplanner/planner -v2 -d ") + domain + std::string(" -p ") + path + std::string("/planningfiles/lastPlan.pddl"));
  size_t pos = 0;
  std::vector<std::string> tokens;

  if(outPlan.length() < 2){
    ROS_ERROR("HTN_PLAN_SERVER: Plan not found");
    return false;
  }



  while((pos = outPlan.find(delimiter)) != std::string::npos) {
    token = outPlan.substr(0, pos);
    if(token.length() > 1){
      tokens.push_back(token.substr(0, token.length()-1));
    }
    outPlan.erase(0, pos + delimiter.length());
  }
  tokens.push_back(outPlan.substr(0, outPlan.length()-1));
  resp.plan = tokens;
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "htn_planner_server");
  ros::NodeHandle nhandler;

  path = ros::package::getPath("ros_htn");
  domain = path + "/planningfiles/domain.pddl";

  ROS_INFO_STREAM(domain);

  ros::ServiceServer planService = nhandler.advertiseService("htn_planner_server", plan);

  ROS_INFO("Server Ready");
  ros::spin();

  return 0;
}
