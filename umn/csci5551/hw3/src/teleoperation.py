#!/usr/bin/env python
#
# This code contains the getKey function, which will get a keypress for you. You'll need to write the rest of the teleoperation node yourself.
# Remember the typical ROS node setup: init the node, set up the publisher, run a loop (probably getting the key press every time you loop), then publish messages based off of that.


import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

MAX_LINEAR_VEL = 0.25
MAX_ANGULAR_VEL = 0.5

def incLinearVel(cur_vel, value):
  if cur_vel + value > MAX_LINEAR_VEL:
    return MAX_LINEAR_VEL
  elif cur_vel + value < -MAX_LINEAR_VEL:
    return -MAX_LINEAR_VEL
  else:
    return cur_vel + value

def incAngularVel(cur_vel, value):
  if cur_vel + value > MAX_ANGULAR_VEL:
    return MAX_ANGULAR_VEL
  elif cur_vel + value < -MAX_ANGULAR_VEL:
    return -MAX_ANGULAR_VEL
  else:
    return cur_vel + value
  

def getKey(key_timeout):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
  if rlist:
      key = sys.stdin.read(1)
  else:
      key = ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('hw3_teleoperation', anonymous=True)
  rate = rospy.Rate(20)
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  vel_msg = Twist()
  
  while(1):
    key = getKey(0.1)
    # If keypress is not empty, print out the key.
    if key != '':
      print(key)
      if key == 'w':
        vel_msg.linear.x = incLinearVel(vel_msg.linear.x, .1)
      elif key == 's':
        vel_msg.linear.x = incLinearVel(vel_msg.linear.x, -.1)
      elif key == 'a':
        vel_msg.angular.z = incAngularVel(vel_msg.angular.z, .1)
      elif key == 'd':
        vel_msg.angular.z = incAngularVel(vel_msg.angular.z, -.1)
      else:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
    # If keypress is Crtl+C, break loop and exit.
    if key == '\x03':
      break
    pub.publish(vel_msg)
    rate.sleep()
    
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
