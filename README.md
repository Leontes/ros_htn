# ROS_HTN
ROS_HTN is a Hierarchical Tasks Planner for the Robotic Operative System (ROS). ROS_HTN contains the functionality needed to solve planning problems in PDDL 2.1 using the HPDL extension [\[Castillo2006\]](#castillo2006) in a ROS nodes architecture. ROS_HTN contains 2 nodes: **plan_service_node** the ros service responsible for calling the HTN planner and return a plan, and **domain_updater_node** the node used to update the planner's domain.

## Table of Contents
1. [Requirements](#requirements)
2. [Installation](#installation)
3. [Usage](#usage)


## Requirements <a id="requirements"></a>
* Ubuntu 16.04
* ROS Kinetic Kame
* GCC with C++11

## Installation <a id="installation"></a>
Starting in a ROS Workspace folder, type:
```bash
cd src/
git clone https://github.com/Leontes/ros_htn.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Usage <a id="usage"></a>
ROS_HTN comes with a launch file ready to be executed and used. This launch will execute ros_core and the plan_service and domain_updater nodes when used setting as default domain a domain passed as the launch argument. If no planning domain is given ROS_HTN will use by default the **domain.pddl** file located in the **planningfiles** folder. When passing a new domain as input you must provide the full path to the new file in order to make ros_htn correctly set the new domain file. The domain_updater node is subscribed to the _/dom_updater_ topic to receive new domains. Launch file can be executed from the command line as follows:

```bash
roslaunch ros_htn htn_server.launch domain:="pathtodomain/newdomain.pddl"
```

Para mandar un problema al servidor primero tiene que estar correctamente formateado. El formato esta definido en el fichero PlanningProblem.msg y consta de 3 partes: objetos del mundo, estado inicial y objetivo. Los objectos del mundo es un array de pares <objeto, tipo de objeto> con la definición de los objetos del mundo. El estado inicial es una lista de predicados en formato PDDL. En este formato los fluentes booleanos se definen como (predicado objeto(s)) mientras que los fluentes numericos se definen como ((predicado objeto(s)) valor). Por ultimo los objetivos es una lista de tareas a resolver tambien en formato PDDL. Estas tareas se definen como (area argumento(s)). El archivo planner_test_client.cpp es un codigo de ejemplo que contiene un problema para el dominio que se encuentra dentro de la carpeta planningfiles.

```c++
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
```

## References
<a id="castillo2006"></a> [Castillo2006](http://www.aaai.org/Papers/ICAPS/2006/ICAPS06-007.pdf): Castillo, L. A., Fernández-Olivares, J., Garcia-Perez, O., & Palao, F. (2006, June). Efficiently Handling Temporal Knowledge in an HTN Planner. In ICAPS (pp. 63-72).
