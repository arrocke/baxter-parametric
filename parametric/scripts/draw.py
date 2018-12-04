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

def mult(s, v):
  return geometry_msgs.msg.Vector3(
    s * v.x,
    s * v.y,
    s * v.z
  )

def add(p, v):
  return geometry_msgs.msg.Point(
    p.x + v.x,
    p.y + v.y,
    p.z + v.z
  )

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
    self.max_x = rospy.get_param("/parametric/max_x")
    self.max_y = rospy.get_param("/parametric/max_y")

    print self.move_group.get_joints()

  def draw(self, x_fn, y_fn, a, b, step):
    sample = self.__generateSample(x_fn, y_fn, a, b, step)
    self.__draw(sample)

  def __generateSample(self, x_fn, y_fn, a, b, step):
    sample = []
    step_count = int((b - a) / step) + 1
    max_x_sample = 0
    max_y_sample = 0

    # Generate the orientation.
    q = quaternion_from_euler(0, math.pi, 0)
    orientation = geometry_msgs.msg.Quaternion(q[0], q[1], q[2], q[3])

    # Generate sample of points from equations.
    for i in range(step_count):
      t = a + i * step
      point = (x_fn(t), y_fn(t))
      sample.append(point)
      # Track the max x and y values.
      max_x_sample = max(max_x_sample, abs(point[0]))
      max_y_sample = max(max_y_sample, abs(point[1]))

    # Map each sample point to robot coordinates.
    for i in range(step_count):
      point = sample[i]
      x = (point[0] / max_x_sample * self.max_x)
      y = (point[1] / max_y_sample * self.max_y)
      pose = geometry_msgs.msg.Pose()
      pose.orientation = orientation
      pose.position = add(
        add(self.origin, mult(x, self.x_direction)),
        mult(y, self.y_direction)
      )
      sample[i] = pose
    return sample

  def __draw(self, sample):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = math.pi / 4 
    joint_goal[1] = -math.pi / 4 
    joint_goal[2] = 0
    joint_goal[3] = 3 * math.pi / 8 
    joint_goal[4] = 0
    joint_goal[5] = 3 * math.pi / 8 
    joint_goal[6] = -math.pi / 2

    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    e0_constraint = moveit_msgs.msg.JointConstraint()
    e0_constraint.position = 0
    e0_constraint.weight = 1
    e0_constraint.tolerance_above = .1
    e0_constraint.tolerance_below = .1 
    e0_constraint.joint_name = "right_e0"

    w0_constraint = moveit_msgs.msg.JointConstraint()
    w0_constraint.position = 0
    w0_constraint.weight = 1
    w0_constraint.tolerance_above = .1
    w0_constraint.tolerance_below = .1 
    w0_constraint.joint_name = "right_w0"

    constraints = moveit_msgs.msg.Constraints()
    constraints.joint_constraints.append(e0_constraint)
    constraints.joint_constraints.append(w0_constraint)
    self.move_group.set_path_constraints(constraints)


    # Move to start position.
    self.move_group.go(sample[0], wait=True)
    self.move_group.stop()

    # Draw the cartesian path.
    (plan, fraction) = self.move_group.compute_cartesian_path(
      sample,   # waypoints to follow
      0.01,        # eef_step
      0.0          # jump_threshold
    )
    print "Path fraction:"
    print fraction
    self.move_group.execute(plan, wait=True)
    self.move_group.stop()

d = ParametricDrawer()
def x_fn(t):
  return math.cos(t)
def y_fn(t):
  return math.sin(t)
d.draw(x_fn, y_fn, 0, 2 * math.pi, 2 * math.pi / 100)