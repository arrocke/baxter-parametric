#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import *
from tf.transformations import *

moveit_commander.roscpp_initialize(['joint_states:=/robot/joint_states'])
rospy.init_node('moveit_baxter_example', anonymous=True)

# Instantiate a RobotCommander object.  This object is
# an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("right_arm")

# Planning to a Pose goal
waypoints = []
waypoints.append(group.get_current_pose().pose)

def generatePose(x, y, z):
  q = quaternion_from_euler(0, 0, 0)
  pose_goal = Pose()
  pose_goal.orientation = Quaternion(q[0], q[1], q[2], q[3])
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z 
  return pose_goal

# waypoints.append(generatePose(0.25,0.25,0))
# waypoints.append(generatePose(0.6,0.25,0))
# waypoints.append(generatePose(0.6,-0.25,0))
# waypoints.append(generatePose(0.25,-0.25,0))

# (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0) 
pose = generatePose(0.5, 0.25, 0)
print pose
group.set_pose_target(pose)
plan = group.plan()

if not plan.joint_trajectory.points:
    print "[ERROR] No trajectory found"
else:
    group.go(wait=True)