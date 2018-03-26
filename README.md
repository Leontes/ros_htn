# ROS_HTN
ROS_HTN is a Hierarchical Tasks Planner for the Robotic Operative System (ROS). ROS_HTN contains the functionality needed to solve planning problems in PDDL 2.1 using the HPDL extension [\[Castillo2006\]](#castillo2006) in a ROS nodes architecture. ROS_HTN contains 2 nodes: **plan_service_node** the ros service responsible for calling the HTN planner and return a plan, and **domain_updater_node** the node used to update the planner's domain.

## Table of Contents
1. [Requirements](#requirements)
2. [Installation](#installation)

## Requirements <a id="requirements"></a>
* Ubuntu 16.04
* ROS Kinetic Kame
* GCC with C++11

## Installation <a id="installation"></a>
Starting in a ROS Workspace folder, type:
'''bash
cd src/
git clone https://github.com/Leontes/ros_htn.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
'''




## References
<a id="castillo2006"></a> [Castillo2006](http://www.aaai.org/Papers/ICAPS/2006/ICAPS06-007.pdf): Castillo, L. A., Fernández-Olivares, J., Garcia-Perez, O., & Palao, F. (2006, June). Efficiently Handling Temporal Knowledge in an HTN Planner. In ICAPS (pp. 63-72).
