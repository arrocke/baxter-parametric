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

PEN_LENGTH = 0.053975

# This class calibrates Baxter to be able to draw on a surface.
# Run the start method to begin.
# Move the arm to the (x, -y) position and press the long button on the cuff.
# Then move the arm to the (-x, -y) position and press the same button.
# Finally move the arm to the (-x, y) position and press the same button.
# This class will install the drawing coordinate frame into the parameter server.
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
      # Calibrate after getting 3 points.
      if len(self.points) == 3:
        self.btn.state_changed.disconnect(self.__get_point)
        self.__calibrate(self.points)

  def __calibrate(self, points):
    p1 = points[0].position
    p2 = points[1].position
    p3 = points[2].position

    def mag(u):
      return math.sqrt(u.x ** 2 + u.y ** 2 + u.z ** 2)

    def unit(u):
      l = mag(u)
      return geometry_msgs.msg.Vector3(u.x / l, u.y / l, u.z / l)

    def cross(u, v):
      return geometry_msgs.msg.Vector3(
        u.y * v.z - u.z * v.y,
        u.z * v.x - u.x * v.z,
        u.x * v.y - u.y * v.x
      )

    def dot(u, v):
      return u.x * v.x + u.y * v.y + u.z * v.z

    def vector(p1, p2):
      return geometry_msgs.msg.Vector3(
        p2.x - p1.x,
        p2.y - p1.y,
        p2.z - p1.z
      ) 

    # Coordinate frame
    u = unit(vector(p2, p1))  # u and v are both vectors in the drawing plane
    v = unit(vector(p2, p3))
    # The origin is halfway between p1 and p3
    origin = geometry_msgs.msg.Point(
      (p1.x + p3.x) / 2,
      (p1.y + p3.y) / 2,
      (p1.z + p3.z) / 2
    )
    # z points out of the drawing plane
    z = cross(u, v)
    x = u
    # y is orthogonal to x and x
    y = cross(z, x)

    # Bounds
    w = vector(origin, p1)  # Vector from origin to corner
    x_bound = dot(w, x)     # Size of w that is in the x direction
    y_bound = dot(w, y)     # Size of w that is in the y direction

    rospy.set_param("/parametric/origin", {
      "x": origin.x,
      "y": origin.y,
      "z": origin.z
    })
    rospy.set_param("/parametric/x_direction", {
      "x": x.x,
      "y": x.y,
      "z": x.z
    })
    rospy.set_param("/parametric/y_direction", {
      "x": y.x,
      "y": y.y,
      "z": y.z
    })
    rospy.set_param("/parametric/z_direction", {
      "x": z.x,
      "y": z.y,
      "z": z.z
    })
    rospy.set_param("/parametric/max_x", x_bound)
    rospy.set_param("/parametric/max_y", y_bound)


    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0, math.pi, 0)
    pose_goal.orientation.w = q[0]
    pose_goal.orientation.x = q[1]
    pose_goal.orientation.y = q[2]
    pose_goal.orientation.z = q[3]
    pose_goal.position = origin 

    self.move_group.go(pose_goal, wait=True)
    self.move_group.stop()

  def verify(self):
    origin = rospy.get_param("/parametric/origin")
    x_dir = rospy.get_param("/parametric/x_direction")

    p1 = {
      "x": origin["x"] + x_dir["x"],
      "y": origin["y"] + x_dir["y"],
      "z": origin["z"] + x_dir["z"]
    }

    joint_goal = [
      math.pi / 4,
      -math.pi / 4,
      0,
      3 * math.pi / 8,
      0,
      3 * math.pi / 8,
      -math.pi / 2
    ]
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0, math.pi, 0)
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
    q = quaternion_from_euler(0, math.pi, 0)
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
v.start()