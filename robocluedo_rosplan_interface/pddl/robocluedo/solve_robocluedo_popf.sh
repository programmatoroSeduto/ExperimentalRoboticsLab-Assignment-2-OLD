#! /bin/bash

timeout 10 /root/ros_ws/src/ROSPlan/rosplan_planning_system/common/bin/popf -l3 /root/ros_ws/src/ExperimentalRoboticsLab-Assignment-2/robocluedo_rosplan_interface/pddl/robocluedo/robocluedo_domain.pddl /root/ros_ws/src/ExperimentalRoboticsLab-Assignment-2/robocluedo_rosplan_interface/pddl/robocluedo/robocluedo_problem.pddl > popf_plan.pddl
