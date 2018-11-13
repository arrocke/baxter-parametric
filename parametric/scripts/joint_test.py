#!/usr/bin/env python
import rospy
from baxter_interface import Limb
import math

L0 = 0.27035
L1 = 0.06900
L2 = 0.36435
L3 = 0.06900
L4 = 0.37429
L5 = 0.01000
L6 = 0.38735
LH = math.sqrt(L2 * L2 + L3 * L3)
LK = math.sqrt(L4 * L4 + L5 * L5)

TH2OFF = math.atan(L3 / L2)
TH3OFF = math.atan(L5 / L4)

def run():
  rospy.init_node("parametric_joints")

  arm = Limb("right")


  x = 0 
  y = 0 
  z = 0

  t1 = math.atan2(y, x) 

  d = math.sqrt(x ** 2 + y ** 2)
  pz = (L6 + z) - L0
  pd = d - L1
  p = math.sqrt(pz ** 2 + pd ** 2)
  a = math.atan(pz / pd)
  b = math.acos((-(p ** 2) + LH ** 2 + LK ** 2) / (2 * LH * LK))
  c = LK * b / p

  t2 = 1
  t3 = 1
  t4 = math.pi / 2 + t2 - t3

  print t1
  print t2
  print t3
  print t4

  joints = arm.joint_angles()
  joints["right_s0"] = math.pi / 4 - t1
  joints["right_s1"] = - TH2OFF - t2
  joints["right_e0"] = 0
  joints["right_e1"] = TH2OFF + t3 + TH3OFF
  joints["right_w0"] = 0
  joints["right_w1"] = TH3OFF + t4
  joints["right_w2"] = 0

  arm.move_to_joint_positions(joints)
  joints = arm.joint_angles()

  print joints
  print arm.endpoint_pose()

  rospy.spin()

if __name__ == '__main__':
  run()


