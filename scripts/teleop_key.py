#!/usr/bin/python

import rospy
import sys, select, termios, tty
import socket
import time
import tf
from sensor_msgs.msg import JointState
import os

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

def callback(data):
  if rospy.get_param('teleop_marker'):
    #time.sleep(1)
    ######  Find the target location, cube_target frame  ######
    try:
      (trans_ct, rot_ct) = listener.lookupTransform('/base', '/cube_target', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("Fail to find the transformation of target")
      #continue

    ######  Find the end_point location, grasping frame  ######
    try:
      (trans_g, rot_g) = listener.lookupTransform('/base', '/tool0', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("Fail to find the transformation of tool0")
      #continue
    print(trans_ct[0])
    
    dis = ((trans_g[0] - trans_ct[0])**2 + (trans_g[1] - trans_ct[1])**2)**0.5
    print(dis)

    key = getKey()
    if dis > 0.1 and (trans_g[0]**2 + trans_g[1]**2)**0.5 > 0.7:     
      rospy.loginfo("Failed and out of the boundary!")
      s.send("stopl(1.0)" + "\n")
      time.sleep(2)
      s.send("movel(p[0.240, -0.11, 0.43, 1.2003, -2.9096, -0.0225])" + "\n")
      os.system('rosparam delete /rotation_marker')
      os.system('rosparam delete /trans_marker')
      os.system('rosparam delete /teleop_marker')
    elif dis > 0.1 and key == 'h':
      rospy.loginfo("Turn left")
      s.send("speedl([0.03, 0.03, 0.0, 0.0, 0.0, 0.0], 1.0, 0.3)" + "\n")
    elif dis > 0.1 and key == 'l':
      rospy.loginfo("Turn right")
      s.send("speedl([0.03, -0.03, 0.0, 0.0, 0.0, 0.0], 1.0, 0.3)" + "\n")
    elif dis > 0.1 and key == 'q':
      rospy.loginfo("Stop")
      s.send("stopl(1.0)" + "\n")
    elif dis <= 0.1:
      rospy.loginfo("The robot will have the control.")
      rospy.set_param('teleop_marker', False)
      rospy.set_param('trans_marker', True)

def teleop_key():
  rospy.init_node('teleop_key', anonymous=True)
  rospy.Subscriber('/joint_states', JointState, callback)
  
  HOST = "192.168.2.75"
  PORT = 30002
  global s
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  
  global listener
  listener = tf.TransformListener()

  rospy.spin()

if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.set_param('trans_marker', False)
  rospy.set_param('teleop_marker', True)
  teleop_key()
