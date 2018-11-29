#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from baxter_interface import DigitalIO
from tf.transformations import quaternion_from_euler
from math import pi

PEN_LENGTH = 0.053975

class Calibration:
  def __init__(self):
    sys.argv.append("joint_states:=/robot/joint_states")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("parametric_calibrater", anonymous=True)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "right_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.btn = DigitalIO("right_upper_button")

  def start(self):
    self.btn.state_changed.connect(self.__get_point)
    self.points = []
    rospy.spin()

  def __get_point(self, down):
    if down:
      pose = self.move_group.get_current_pose().pose
      print pose
      self.points.append(pose)
      if len(self.points) == 3:
        self.btn.state_changed.disconnect(self.__get_point)
        self.__calibrate(self.points)

  def __calibrate(self, points):
    p1 = points[0].position
    p2 = points[1].position
    p3 = points[2].position

    origin = {
      "x": (p1.x + p3.x) / 2,
      "y": (p1.y + p3.y) / 2,
      "z": (p1.z + p3.z) / 2
    }

    rospy.set_param("/parametric/origin", origin)

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

    self.verify()

  def verify(self):
    origin = rospy.get_param("/parametric/origin")
    x_dir = rospy.get_param("/parametric/x_direction")

    p1 = {
      "x": origin["x"] + x_dir["x"],
      "y": origin["y"] + x_dir["y"],
      "z": origin["z"] + x_dir["z"]
    }

    joint_goal = [
      pi / 4,
      -pi / 4,
      0,
      3 * pi / 8,
      0,
      3 * pi / 8,
      -pi / 2
    ]
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0, pi, 0)
    pose_goal.orientation.w = q[0]
    pose_goal.orientation.x = q[1]
    pose_goal.orientation.y = q[2]
    pose_goal.orientation.z = q[3]
    pose_goal.position.x = origin["x"]
    pose_goal.position.y = origin["y"] 
    pose_goal.position.z = origin["z"] 

    self.move_group.go(pose_goal, wait=True)
    self.move_group.stop()

    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0, pi, 0)
    pose_goal.orientation.w = q[0]
    pose_goal.orientation.x = q[1]
    pose_goal.orientation.y = q[2]
    pose_goal.orientation.z = q[3]
    pose_goal.position.x = p1["x"]
    pose_goal.position.y = p1["y"] 
    pose_goal.position.z = p1["z"] 
    
    self.move_group.set_pose_target(pose_goal)
    self.move_group.go()

v = Calibration()
v.verify()