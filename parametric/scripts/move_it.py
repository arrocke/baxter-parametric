#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

# Remap joint_states topic for MoveIt.
sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
  '/move_group/display_planned_path',
  moveit_msgs.msg.DisplayTrajectory,
  queue_size=20
)
current_pose = move_group.get_current_pose().pose

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = pi / 4 
joint_goal[1] = -pi / 4 
joint_goal[2] = 0
joint_goal[3] = 3 * pi / 8 
joint_goal[4] = 0
joint_goal[5] = 3 * pi / 8 
joint_goal[6] = -pi / 2

# # The go command can be called with joint values, poses, or without any
# # parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# # Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

waypoints = []

def generatePose(x, y):
  pose_goal = geometry_msgs.msg.Pose()
  q = quaternion_from_euler(0, pi, 0)
  pose_goal.orientation.w = q[0]
  pose_goal.orientation.x = q[1]
  pose_goal.orientation.y = q[2]
  pose_goal.orientation.z = q[3]
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = -0.2 
  return pose_goal

waypoints.append(generatePose(0.7, 0))
waypoints.append(generatePose(0.7, -0.4))
waypoints.append(generatePose(0.4, -0.4))
waypoints.append(generatePose(0.4, 0))
waypoints.append(generatePose(0.7, 0))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

print fraction

move_group.execute(plan, wait=True)
move_group.stop()