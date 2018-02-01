#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
import tf
from sensor_msgs.msg import JointState
import numpy as np

def callback(data):

  try:
    (trans_tool0, rot_tool0) = listener.lookupTransform('/base', '/tool0', rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("Fail to find the transformation of tool0")

  print(trans_tool0[0]) 
  pub.publish(Vector3(trans_tool0[0], trans_tool0[1], trans_tool0[2]))

def tool0_position_pub():
  global pub
  pub = rospy.Publisher('tool0_position', Vector3, queue_size=10)
  rospy.init_node('tool0_postion_pub', anonymous=True)
  rospy.Subscriber('/joint_states', JointState, callback)

  global listener
  listener = tf.TransformListener()

  rospy.spin()
    
if __name__ == '__main__':
  tool0_position_pub()
