#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("parametric_debugger", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
  rospy.loginfo(move_group.get_current_joint_values())
  rate.sleep()
