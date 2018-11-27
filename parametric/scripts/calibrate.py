#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from baxter_interface import DigitalIO

sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("parametric_calibrater", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

btn = DigitalIO("right_upper_button")
points = []

def get_point(v):
  if v:
    pose = move_group.get_current_pose().pose
    print pose
    points.append(pose)
    if len(points) == 3:
      btn.state_changed.disconnect(get_point)
      calibrate()

btn.state_changed.connect(get_point)

def calibrate():
  p1 = points[0].position
  p2 = points[1].position
  p3 = points[2].position

  rospy.set_param("/parametric/origin", {
    "x": (p1.x + p3.x) / 2,
    "y": (p1.y + p3.y) / 2,
    "z": (p1.z + p3.z) / 2
  })

  rospy.set_param("/parametric/x_direction", {
    "x": (p2.x - p1.x) / 2,
    "y": (p2.y - p1.y) / 2,
    "z": (p2.z - p1.z) / 2
  })

  rospy.set_param("/parametric/y_direction", {
    "x": (p3.x - p2.x) / 2,
    "y": (p3.y - p2.y) / 2,
    "z": (p3.z - p2.z) / 2
  })

  rospy.signal_shutdown("Finished")

rospy.spin()