#include "ros/ros.h"
#include <ros_htn/GetPlan.h>
#include <ros_htn/Object.h>
#include <ros_htn/PlanningProblem.h>
#include <string>
#include <vector>
#include <fstream>


int main(int argc, char *argv[]) {

  ros_htn::Object obj;
  std::vector<ros_htn::Object> objectsVec;

  obj.objName = "general";
  obj.objType = "lander";
  objectsVec.push_back(obj);

  obj.objName = "rover0";
  obj.objType = "rover";
  objectsVec.push_back(obj);

  obj.objName = "rover0store";
  obj.objType = "store";
  objectsVec.push_back(obj);

  obj.objName = "waypoint0";
  obj.objType = "waypoint";
  objectsVec.push_back(obj);

  obj.objName = "waypoint1";
  obj.objType = "waypoint";
  objectsVec.push_back(obj);

  obj.objName = "waypoint2";
  obj.objType = "waypoint";
  objectsVec.push_back(obj);

  obj.objName = "camera0";
  obj.objType = "camera";
  objectsVec.push_back(obj);

  obj.objName = "objective0";
  obj.objType = "objective";
  objectsVec.push_back(obj);

  ros_htn::PlanningProblem problem;

  problem.objects = objectsVec;

  std::vector<std::string> state;

  state.push_back("(at_soil_sample waypoint1)");
  state.push_back("(at_rock_sample waypoint2)");
  state.push_back("(visible_from objective0 waypoint2)");
  state.push_back("(calibration_target camera0 objective0)");
  state.push_back("(in_sun waypoint2)");
  state.push_back("(at_lander general waypoint0)");
  state.push_back("(channel_free general)");
  state.push_back("(visible waypoint0 waypoint1)");
  state.push_back("(visible waypoint0 waypoint2)");
  state.push_back("(store_of rover0store rover0)");
  state.push_back("(empty rover0store)");
  state.push_back("(equipped_for_rock_analysis rover0)");
  state.push_back("(equipped_for_soil_analysis rover0)");
  state.push_back("(equipped_for_imaging rover0)");
  state.push_back("(visible waypoint1 waypoint2)");
  state.push_back("(on_board camera0 rover0)");
  state.push_back("(supports camera0 colour)");
  state.push_back("(supports camera0 high_res)");
  state.push_back("(supports camera0 low_res)");
  state.push_back("(= (energy rover0) 5000)");
  state.push_back("(= (recharge-rate rover0) 5)");
  state.push_back("(at rover0 waypoint0)");
  state.push_back("(available rover0)");
  state.push_back("(can_traverse rover0 waypoint0 waypoint1)");
  state.push_back("(can_traverse rover0 waypoint0 waypoint2)");
  state.push_back("(can_traverse rover0 waypoint1 waypoint2)");

  problem.initState = state;


  std::vector<std::string> tasks;

  tasks.push_back("(sample rover0 waypoint1)");
  tasks.push_back("(sample rover0 waypoint2)");
  tasks.push_back("(photo rover0 objective0 high_res)");

  problem.goal = tasks;


  ros_htn::GetPlan rqt;
  rqt.request.problem = problem;




  ros::init(argc, argv, "planner_test_client");
  ros::NodeHandle nhandler;
  ros::ServiceClient client = nhandler.serviceClient<ros_htn::GetPlan>("htn_planner_server");

  if (client.call(rqt)){
    std::vector<std::string> outPlan = rqt.response.plan;
    for(unsigned int i = 0; i < outPlan.size(); i++){
      ROS_INFO_STREAM(outPlan[i]);
    }
  }
  else{
    ROS_ERROR("Error conecting with the planning service");
    return -1;
  }

  return 0;
}
