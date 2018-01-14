#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import Char
import socket

#def callback(tele_c):
#  listener = tf.TransformListener()
#  ######  Find the target location, cube_target frame  ######
#  try:
#    (trans_ct, rot_ct) = listener.lookupTransform("world", "cube_target", rospy.Time(0))
#  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#    continue
#  ######  Find the end_point location, grasping frame  ######
#  try:
#    (trans_g, rot_g) = listener.lookupTransform("world", "grasping_frame", rospy.Time(0))
#  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#    continue

#  dis = ((trans_g[0] - trans_ct[0])**2 + (trans_g[1] - trans_ct[1])**2)**0.5
#  if dis > 0.1 and tele_c.data == 'L'
#    rospy.loginfo("Turn left")
#    s.send("speedl([0.05, 0.05, 0.0, 0.0, 0.0, 0.0], 1.0, 0.2)" + "\n")
#  elif dis > 0.1 and tele_c.data == 'R'
#    rospy.loginfo("Turn right")
#    s.send("speedl([0.05, -0.05, 0.0, 0.0, 0.0, 0.0], 1.0, 0.2)" + "\n")
#  elif dis <= 0.1
#    rospy.loginfo("The robot will have control.")
#    s.send("stopl(1.0)" + "\n")
#    trigger = 'T'
#    pub.publish(trigger)
#    rospy.sleep(30.)

def teleop_BCI():
  rospy.init_node('teleop_BCI', anonymous=True)
#  rospy.Subscriber('command_BCI', Char, callback)
#  global pub
#  pub = rospy.Publisher('trigger_auto', Char, queue_size=1)
  
  HOST = "192.168.2.75"
  PORT = 30002
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
#  y should add 0.04 for compensate
#  the fourth to sixth para is rotate vector on control board, not RPY
  
  s.send("movel(p[0.637, -0.216, 0.382, 0.962, -2.949, -0.042], 1.0, 0.15, 0)" + "\n") 
#  rospy.spin()
    

if __name__ == '__main__':
  teleop_BCI()