#!/usr/bin/env python
import rospy
import struct
from baxter_interface import Limb
from std_msgs.msg import Header
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion
)
from baxter_core_msgs.srv import (
  SolvePositionIK,
  SolvePositionIKRequest
)

def test():
  rospy.init_node('ik_test')
  ns = "ExternalTools/right/PositionKinematicsNode/IKService" 
  iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
  ikreq = SolvePositionIKRequest()
  ikreq.pose_stamp.append(PoseStamped(
    header = Header(stamp = rospy.Time.now(), frame_id = "base"),
    pose = Pose(
      position = Point(
        x = rospy.get_param("/parametric/drawing_frame/origin/x"),
        y = rospy.get_param("/parametric/drawing_frame/origin/y"),
        z = rospy.get_param("/parametric/drawing_frame/origin/z")
      ),
      orientation = Quaternion(
        x = rospy.get_param("/parametric/gripper_orientation/x"),
        y = rospy.get_param("/parametric/gripper_orientation/y"),
        z = rospy.get_param("/parametric/gripper_orientation/z"),
        w = rospy.get_param("/parametric/gripper_orientation/w")
      )
    )
  ))

  print ikreq

  try:
    rospy.wait_for_service(ns, 5.0)
    resp = iksvc(ikreq)
  except (rospy.ServiceException, rospy.ROSException), e:
    rospy.logerr("Service call failed: %s" % (e,))
    return 1
  
  resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
  if (resp_seeds[0] != resp.RESULT_INVALID):
    limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))    
    print limb_joints

    arm = Limb('right')
    arm.move_to_joint_positions(limb_joints)
  else:
    print "Invalid target"

if __name__ == '__main__':
  test()