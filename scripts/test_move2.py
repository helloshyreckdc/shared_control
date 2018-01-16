#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import Char
from std_msgs.msg import Float64
import socket
from sensor_msgs.msg import JointState
import math
from numpy import *
import time

count = 1
def callback(data):
 # nonlocal count
  if rospy.get_param('rotation_marker'):    
    global count 
    if count == 1:
      #global count
        joint1 = data.position[0]
        joint2 = data.position[1]
        joint3 = data.position[2]
        joint4 = data.position[3]
        joint5 = data.position[4]
        joint6 = data.position[5]
        
        try:
          (trans_tool0, rot_tool0) = listener.lookupTransform('/base', '/grasping_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print("Fail to find the transformation of tool0")
    
        try:
          (trans_target, rot_target) = listener.lookupTransform('/base', '/cube_target', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print("Fail to find the transformation of target")
    
        matrix_tool0 = tf.transformations.quaternion_matrix(rot_tool0)
        matrix_target = tf.transformations.quaternion_matrix(rot_target)
    
        matrix_tool001 = matrix_tool0[0][2]
        matrix_tool011 = matrix_tool0[1][2]
        matrix_tool021 = matrix_tool0[2][2]
        matrix_target00 = matrix_target[0][0]
        matrix_target10 = matrix_target[1][0]
        matrix_target20 = matrix_target[2][0]
        matrix_target01 = matrix_target[0][1]
        matrix_target11 = matrix_target[1][1]
        matrix_target21 = matrix_target[2][1]
        matrix_target02 = matrix_target[0][2]
        matrix_target12 = matrix_target[1][2]
        matrix_target22 = matrix_target[2][2]
        tool0_z = mat([[matrix_tool001], [matrix_tool011], [matrix_tool021]])
        print("tool0_z", tool0_z)
        target_1 = mat([[matrix_target00], [matrix_target10], [matrix_target20]])
        target_2 = mat([[matrix_target01], [matrix_target11], [matrix_target21]])
        target_3 = mat([[matrix_target02], [matrix_target12], [matrix_target22]])
    
        a = tool0_z.T*target_1
        b = tool0_z.T*target_2
        c = tool0_z.T*target_3
        listABC = [abs(a[0][0]), abs(b[0][0]), abs(c[0][0])]
        d = max(listABC)
        index = listABC.index(d)
        rad = math.acos(d)
 #       tool0_z1 = mat([[matrix_tool001], [math.cos(rad)*matrix_tool011 - math.sin(rad)*matrix_tool021], [math.sin(rad)*matrix_tool011 + math.cos(rad)*matrix_tool021]])
        tool0_z1 = mat([[matrix_tool0[0][0], matrix_tool0[0][1], matrix_tool0[0][2]], [matrix_tool0[1][0], matrix_tool0[1][1], matrix_tool0[1][2]], [matrix_tool0[2][0], matrix_tool0[2][1], matrix_tool0[2][2]]]) * mat([[0, -math.sin(rad), math.cos(rad)]]).T
        print("tool0_z1", tool0_z1)
        if index == 0:
          a_after_rot = tool0_z1.T*target_1
          print(a_after_rot[0][0])
          print(a[0][0])
          if abs(a_after_rot[0][0])>abs(a[0][0]):
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6+rad) + "])" + "\n")
            print("a +")
          else:
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6-rad) + "])" + "\n")
            print("a -")
        elif index == 1:
          b_after_rot = tool0_z1.T*target_2
          print(b_after_rot[0][0])
          print(b[0][0])
          if abs(b_after_rot[0][0])>abs(b[0][0]):
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6+rad) + "])" + "\n")
            print("b +")
          else:
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6-rad) + "])" + "\n")
            print("b -")
        elif index == 2:
          c_after_rot = tool0_z1.T*target_3
          print(c_after_rot[0][0])
          print(c[0][0])
          if abs(c_after_rot[0][0])>abs(c[0][0]):
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6+rad) + "])" + "\n")
            print("c +")
          else:
            s.send("movej([" + str(joint1) + ", " + str(joint2) + ", " + str(joint3) + ", " + str(joint4) + ", " + str(joint5) + ", " + str(joint6-rad) + "])" + "\n")
            print("c -")
        print('a=', a)
        print('b=', b)
        print('c=', c)
        print('d=', d)
        print(rad)

        time.sleep(1)
	gripper_width = 48
	pub.publish(gripper_width)
	print("Grasping finished!")
	#time.sleep(4)
	#s.send("movel(p[0.507, -0.471, 0.399, 1.2812, -2.8750, -0.0221])" + "\n")
	#time.sleep(3)
	#gripper_width = 100
	#pub.publish(gripper_width)
	#print("Release finished!")
	#time.sleep(4)
	#s.send("movel(p[0.235, -0.11, 0.43, 1.2003, -2.9096, -0.0225])" + "\n")
	#print("Pick and place finished!")
    
    count = count+1
    # rospy.sleep(300)

def teleop_BCI():
  rospy.init_node('teleop_BCI', anonymous=True)
  rospy.Subscriber('/joint_states', JointState, callback)
  global listener
  listener = tf.TransformListener()
  global pub
  pub = rospy.Publisher('gripper_width', Float64, queue_size=10)
  HOST = "192.168.2.75"
  PORT = 30002
  global s
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((HOST, PORT))
  
#  global pub
#  pub = rospy.Publisher('trigger_auto', Char, queue_size=1)
#  y should add 0.04 for compensate
#  the fourth to sixth para is rotate vector on control board, not RPY
#  s.send("get_target_joint_positions()" + "\n")
#  data = s.recv(1024)
#  print(repr(data))
  rospy.spin()
    

if __name__ == '__main__':
  
  teleop_BCI()
