#!/usr/bin/env python

import rospy
from baxter_interface import(
  DigitalIO,
  Gripper,
  CHECK_VERSION
)

# This program gives control of the gripper to the cuff buttons.
# The long button closes the gripper
# and the round button opens the gripper.

rospy.init_node("parametric_gripper")

close_button = DigitalIO("right_upper_button")
open_button = DigitalIO("right_lower_button")

gripper = Gripper("right", CHECK_VERSION)
gripper.calibrate()

if (not (gripper.calibrated())):
  rospy.logwarn("Gripper calibration failed.")

close_button.state_changed.connect(gripper.close)
open_button.state_changed.connect(gripper.open)

print "Press cuff buttons to control gripper."
rospy.spin()
print "Finished gripper controls."