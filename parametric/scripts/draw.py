#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from baxter_interface import DigitalIO
from tf.transformations import quaternion_from_euler
import math 
import copy
import motion
import vmath

# This class uses the calibration data to draw parametric equations.
# Call the draw method with functions for x and y, minimum and maximum values for t and the step size.
class ParametricDrawer:
  def __init__(self):
    sys.argv.append("joint_states:=/robot/joint_states")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("parametric_drawer", anonymous=True)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "right_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    origin = rospy.get_param("/parametric/origin")
    self.origin = geometry_msgs.msg.Point(origin["x"], origin["y"], origin["z"])
    x_direction = rospy.get_param("/parametric/x_direction")
    self.x_direction = geometry_msgs.msg.Vector3(x_direction["x"], x_direction["y"], x_direction["z"])
    y_direction = rospy.get_param("/parametric/y_direction")
    self.y_direction = geometry_msgs.msg.Vector3(y_direction["x"], y_direction["y"], y_direction["z"])
    z_direction = rospy.get_param("/parametric/z_direction")
    self.z_direction = geometry_msgs.msg.Vector3(z_direction["x"], z_direction["y"], z_direction["z"])
    max_x = rospy.get_param("/parametric/max_x")
    max_y = rospy.get_param("/parametric/max_y")
    self.max_pose = min(max_x, max_y)

  def draw(self, x_fn, y_fn, a, b, steps):
    sample = self.__generateSample(x_fn, y_fn, a, b, steps)
    self.__draw(sample)

  def __generateSample(self, x_fn, y_fn, a, b, steps):
    sample = []
    step_size = (b - a) / (steps - 1)
    max_x_sample = 0
    max_y_sample = 0

    # Generate sample of points from equations.
    for i in range(steps):
      t = a + i * step_size
      point = (x_fn(t), y_fn(t))
      sample.append(point)
      # Track the max x and y values.
      max_x_sample = max(max_x_sample, abs(point[0]))
      max_y_sample = max(max_y_sample, abs(point[1]))

    # Map each sample point to robot coordinates.
    for i in range(steps):
      point = sample[i]
      x = (point[0] / max_x_sample * self.max_pose)
      y = (point[1] / max_y_sample * -self.max_pose)
      sample[i] = vmath.add(
        vmath.add(self.origin, vmath.scale(x, self.x_direction)),
        vmath.scale(y, self.y_direction)
      )
    return sample

  def __draw(self, sample):
    motion.neutral(self.move_group)
    motion.position(self.move_group, sample[0])
    motion.waypoints(self.move_group, sample)
    motion.neutral(self.move_group)
