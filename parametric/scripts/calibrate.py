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
import baxter_external_devices
import motion
import vmath

PEN_LENGTH = 0.053975

# This class calibrates Baxter to be able to draw on a surface.
# Run the start method to begin.
# Move the arm to the (x, -y) position and press the long button on the cuff.
# Then move the arm to the (-x, -y) position and press the same button.
# Finally move the arm to the (-x, y) position and press the same button.
# Then the arm will move to the origin.
# Press w to raise the origin point and s to lower.
# Press d to continue.
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
        self.__calibrate(self.points)

  def __get_vertical(self, origin):
    motion.neutral(self.move_group)

    oldz = origin.z
    origin.z += 0.005
    ch = 0
    while ch != 'd':
      ch = baxter_external_devices.getch()
      if ch == 'w':
        origin.z += 0.005
      elif ch == 's':
        origin.z -= 0.005
      if oldz != origin.z:
        print origin.z
        oldz = origin.z
        motion.position(self.move_group, origin)

    return origin

  def __calibrate(self, points):
    p1 = points[0].position
    p2 = points[1].position
    p3 = points[2].position


    # The origin is halfway between p1 and p3
    origin = geometry_msgs.msg.Point(
      (p1.x + p3.x) / 2,
      (p1.y + p3.y) / 2,
      (p1.z + p3.z) / 2
    )

    origin = self.__get_vertical(origin)    

    # z points out of the drawing plane
    # y is orthogonal to x and x
    z = geometry_msgs.msg.Vector3(0, 0, 1)
    u = vmath.unit(vmath.vector(p2, p1))
    x = vmath.unit(geometry_msgs.msg.Vector3(u.x, u.y, 0))
    y = vmath.cross(z, x)

    # Bounds
    w = vmath.vector(origin, p1)  # Vector from origin to corner
    x_bound = vmath.dot(w, x)     # Size of w that is in the x direction
    y_bound = vmath.dot(w, y)     # Size of w that is in the y direction

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

    self.verify()

    rospy.signal_shutdown("calibration complete")

  def verify(self):
    origin = rospy.get_param("/parametric/origin")
    x_dir = rospy.get_param("/parametric/x_direction")
    y_dir = rospy.get_param("/parametric/y_direction")
    max_x = rospy.get_param("/parametric/max_x")
    max_y = rospy.get_param("/parametric/max_y")

    origin = geometry_msgs.msg.Vector3(origin["x"], origin["y"], origin["z"])
    x_dir = geometry_msgs.msg.Vector3(x_dir["x"], x_dir["y"], x_dir["z"])
    y_dir = geometry_msgs.msg.Vector3(y_dir["x"], y_dir["y"], y_dir["z"])

    print origin
    motion.neutral(self.move_group)
    motion.position(self.move_group, origin)

    motion.neutral(self.move_group)
    x_max = vmath.add(origin, vmath.scale(max_x, x_dir))
    print x_max
    motion.position(self.move_group, x_max)

    motion.neutral(self.move_group)
    y_max = vmath.add(origin, vmath.scale(max_y, y_dir))
    print y_max
    motion.position(self.move_group, y_max)

    motion.neutral(self.move_group)
    print origin
    motion.position(self.move_group, origin)

v = Calibration()
v.start()