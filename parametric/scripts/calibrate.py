#!/usr/bin/env python
import rospy
from baxter_interface import Limb

def calibrate():
  rospy.init_node("parametric_calibrate")

  arm = Limb("right")

  raw_input("Move the drawing arm to the bottom right corner of the drawing surface. Press ENTER to continue...")
  br = arm.endpoint_pose()

  raw_input("Move the drawing arm to the top left corner of the drawing surface. Press ENTER to continue...")
  tl = arm.endpoint_pose()

  rospy.set_param("/parametric/drawing_frame/", {
    "origin": {
      "x": (br["position"].x + tl["position"].x) / 2.0,
      "y": (br["position"].y + tl["position"].y) / 2.0,
      "z": (br["position"].z + tl["position"].z) / 2.0,
    }
  })
  rospy.set_param("/parametric/gripper_orientation", {
    "x": br["orientation"].x,
    "y": br["orientation"].y,
    "z": br["orientation"].z,
    "w": br["orientation"].w
  })

if __name__ == '__main__':
  calibrate()