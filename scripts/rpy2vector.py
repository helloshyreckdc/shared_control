#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import Char
import socket
from sensor_msgs.msg import JointState
import math
from numpy import *
import time

count = 1
def callback(data):
  global count 
  if count == 1:
    try:
      (trans_target, rot_target) = listener.lookupTransform('/base', '/cube_target', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("Fail to find the transformation of target")
    
    s.send("movel(p[" + str(trans_target[0]+0.005) + ", " + str(trans_target[1]+0.05) + ", 0.316, 1.2003, -2.9096, -0.0222])" + "\n")
    print("Finish movement!")
    time.sleep(3)
    rospy.set_param('rotation_marker', True)

  count = count+1
   # rospy.sleep(300)

def teleop_BCI():
  rospy.init_node('teleop_BCI', anonymous=True)
  rospy.Subscriber('/joint_states', JointState, callback)

  global listener
  listener = tf.TransformListener()

  HOST = "192.168.2.75"
  PORT = 30002
  global s
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))

  rospy.spin()
    

if __name__ == '__main__':
  rospy.set_param('rotation_marker', False)
  teleop_BCI()
