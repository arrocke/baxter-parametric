from geometry_msgs.msg import (
  Quaternion,
  Pose
)
from moveit_msgs.msg import (
  Constraints,
  JointConstraint,
)
from tf.transformations import quaternion_from_euler
import math
from copy import deepcopy

e0_constraint = JointConstraint()
e0_constraint.position = 0
e0_constraint.weight = 1
e0_constraint.tolerance_above = .01
e0_constraint.tolerance_below = .01 
e0_constraint.joint_name = "right_e0"

w0_constraint = JointConstraint()
w0_constraint.position = 0
w0_constraint.weight = 1
w0_constraint.tolerance_above = .01
w0_constraint.tolerance_below = .01 
w0_constraint.joint_name = "right_w0"

constraints = Constraints()
constraints.joint_constraints.append(e0_constraint)
constraints.joint_constraints.append(w0_constraint)

q = quaternion_from_euler(0, math.pi, math.pi)
orientation = Quaternion(q[0], q[1], q[2], q[3])

# Common motions for the arm


def neutral(move_group):
  joint_goal = [
    math.pi / 4,
    -math.pi / 4,
    0,
    3 * math.pi / 8,
    0,
    3 * math.pi / 8,
    -math.pi / 2
  ]

  move_group.clear_path_constraints()
  move_group.go(joint_goal, wait=True)
  move_group.stop()

def position(move_group, position):
  pose_goal = Pose()
  pose_goal.orientation = orientation
  pose_goal.position = position 

  move_group.clear_path_constraints()
  move_group.set_path_constraints(deepcopy(constraints))
  move_group.go(pose_goal, wait=True)
  move_group.stop()

def waypoints(move_group, waypoints):
  poses = []
  for point in waypoints:
    pose_goal = Pose()
    pose_goal.orientation = orientation
    pose_goal.position = point 
    poses.append(pose_goal)

  # Draw the cartesian path.
  move_group.clear_path_constraints()
  move_group.set_path_constraints(deepcopy(constraints))
  (plan, fraction) = move_group.compute_cartesian_path(
    poses,   # waypoints to follow
    0.01,        # eef_step
    0.0          # jump_threshold
  )
  print "Path fraction:"
  print fraction
  if (fraction == 1.0):
    move_group.execute(plan, wait=True)
    move_group.stop()
  else:
    print "NO PATH FOUND"
  