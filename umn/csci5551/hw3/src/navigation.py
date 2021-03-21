#!/usr/bin/env python

# Written By Yang He

# Referencing the https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
# I got some ideas from this tutorial (especially the FSM for the ROS prog) but implemented everything by myself.

# Using BUG2 as the planning algorithm, but modified a little bit.

import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import sys, select, termios, tty, math
from enum import Enum

# Settings
MAX_LASER_DIST = 3.0
OBSTACLE_THRESHOLD = 0.12
#SAME_POINT_DIST_THRESHOLD = 0.01
SAME_LINE_DIST_THRESHOLD = 0.015
LINEAR_DIST_THRESHOLD = 0.15
ANGULAR_DIST_THRESHOLD = 0.05
NORMAL_LINEAR_VEL = 0.25
NORMAL_ANGULAR_VEL = 0.7
OBSTACLE_SKIP_SAMPLES = 1000

send_msg_to_service = False

# FSM 
States = Enum('States', ('FixYaw', 'MoveStraight', 'TouchObstacle', 'MoveObstacle', 'Done', 'Fail'))
cur_state = States.Done

# Direction enum
Direction = Enum('Direction', ('FRONT', 'FRONTLEFT', 'LEFT', 'FRONTRIGHT', 'RIGHT'))

# Environmental data
init_x, init_y = 0.0, 0.0 # Starting point - to compute the M-Line
goal_x, goal_y = 0.0, 0.0
cur_x, cur_y, cur_th = 0.0, 0.0, 0.0

laser_each_angle = 0.0175
laser_ranges = []
obstacle_entry_pt = (0.0, 0.0)
obstacle_entry_goal_dist = 0.0
obstacle_pts = []
obstacle_pts_sample_cnt = OBSTACLE_SKIP_SAMPLES

# Publishers
vel_pub = None

def sendMsgToService():
  rospy.wait_for_service('/csci_5551/goal_reached')
  try:
    goal_reached_srv = rospy.ServiceProxy('/csci_5551/goal_reached', Trigger)
    resp_srv = goal_reached_srv()
    return resp_srv.success
  except rospy.ServiceException as e:
    print('Service call failed: {}'.format(e))
  

def minDistAround():
  global laser_ranges, laser_each_angle

  allDist = {};
  for pair in Direction:
    allDist[pair.value] = MAX_LASER_DIST

  for dist in laser_ranges[:31]:
    allDist[Direction.FRONT.value] = min(allDist[Direction.FRONT.value], dist)
  for dist in laser_ranges[330:]:
    allDist[Direction.FRONT.value] = min(allDist[Direction.FRONT.value], dist)

  for dist in laser_ranges[31:81]:
    allDist[Direction.FRONTLEFT.value] = min(allDist[Direction.FRONTLEFT.value], dist)
  for dist in laser_ranges[81:91]:
    allDist[Direction.LEFT.value] = min(allDist[Direction.LEFT.value], dist)

  for dist in laser_ranges[270:280]:
    allDist[Direction.RIGHT.value] = min(allDist[Direction.RIGHT.value], dist)
  for dist in laser_ranges[280:330]:
    allDist[Direction.FRONTRIGHT.value] = min(allDist[Direction.FRONTRIGHT.value], dist)

  return allDist

def diffYaw(cur_pos, tar_pos, cur_yaw):
  diff_yaw = math.atan2(tar_pos[1] - cur_pos[1], tar_pos[0] - cur_pos[0]) - cur_yaw
  return diff_yaw

def fixYaw(diff_yaw):
  scale = 1.0 if math.fabs(diff_yaw) > 1.05 else math.fabs(diff_yaw) / 1.05
  if scale < 0.5: scale = 0.5
  # return NORMAL_ANGULAR_VEL if diff_yaw > 0 else -NORMAL_ANGULAR_VEL
  angular_vel =  NORMAL_ANGULAR_VEL if diff_yaw > 0 else -NORMAL_ANGULAR_VEL
  return scale * angular_vel

# Distance from pt2 to the line formed by pt1 and pt3
def ptDistToLine(pt1, pt2, pt3):
  # Line equation: Ax+By+C = 0 => (y3-y1)x + (x1-x3)y + (x3y1-x1y3)=0
  # Distance of pt2 to the m-line: |Ax0+By0+C| / sqrt(A^2 + B^2)
  line_A, line_B, line_C = pt3[1]-pt1[1], pt1[0]-pt3[0], pt3[0]*pt1[1]-pt1[0]*pt3[1]
  return math.fabs(line_A*pt2[0] + line_B*pt2[1] + line_C) / math.sqrt(line_A**2 + line_B**2) 

def threePointsSameLine(pt1, pt2, pt3):
  # print(pt2_to_line_dist)
  pt2_to_line_dist = ptDistToLine(pt1, pt2, pt3)
  return pt2_to_line_dist <= SAME_LINE_DIST_THRESHOLD

def stateFixYaw():
  global vel_pub, cur_state

  vel_msg = Twist()
  diff_yaw = diffYaw((cur_x, cur_y), (goal_x, goal_y), cur_th)
  if math.fabs(diff_yaw) > ANGULAR_DIST_THRESHOLD: 
    vel_msg.angular.z = fixYaw(diff_yaw)
  else:
    cur_state = States.MoveStraight  
  
  vel_pub.publish(vel_msg)

def stateMoveStraight():
  global send_msg_to_service
  global cur_x, cur_y, goal_x, goal_y
  global vel_pub, cur_state

  vel_msg = Twist()
  goal_dist = math.sqrt((goal_x - cur_x) ** 2 + (goal_y - cur_y) ** 2)
  if goal_dist <= LINEAR_DIST_THRESHOLD:
    print('Goal reached! Current distance: {}'.format(goal_dist))
    print('Goal pos: {}, {}, current pos: {}, {}'.format(goal_x, goal_y, cur_x, cur_y))
    cur_state = States.Done 
    send_msg_to_service = True 

    # send_status = sendMsgToService()
    # if send_status:
    #   print('Message has been sent to the service!')
  else: 
    dists_around = minDistAround() 
    if dists_around[Direction.FRONT.value] <= 2.2 * OBSTACLE_THRESHOLD:
      # If the goal is around the agent now and has a distance less than obstacle detected distance, 
      # then count the case as "goal reached" to avoid computing issues (hard to compute tiny nums) 
      # while handling obstacle moving.
      if goal_dist <= 2.2 * OBSTACLE_THRESHOLD:
        print('Goal reached! Current distance: {}'.format(goal_dist))
        print('Goal pos: {}, {}, current pos: {}, {}'.format(goal_x, goal_y, cur_x, cur_y))
        cur_state = States.Done 
        send_msg_to_service = True 

        # send_status = sendMsgToService()
        # if send_status:
        #   print('Message has been sent to the service!')
        return
      else:
        cur_state = States.TouchObstacle
        print('Encounter the obstacle!')
    else:
      vel_msg.linear.x = NORMAL_LINEAR_VEL 
  vel_pub.publish(vel_msg)
  
  diff_yaw = diffYaw((cur_x, cur_y), (goal_x, goal_y), cur_th)
  if math.fabs(diff_yaw) > ANGULAR_DIST_THRESHOLD:
    cur_state = States.FixYaw

def stateTouchObstacle():
  global cur_x, cur_y, goal_x, goal_y
  global obstacle_entry_pt, obstacle_entry_goal_dist, obstacle_pts_sample_cnt 
  global vel_pub, cur_state

  obstacle_entry_pt = (cur_x, cur_y)
  obstacle_entry_goal_dist = math.sqrt((goal_x - cur_x) ** 2 + (goal_y - cur_y) ** 2)
  obstacle_pts_sample_cnt = OBSTACLE_SKIP_SAMPLES

  # Didn't check goal point in the previous version (worked good except goals that are closed to obs)
  vel_msg = Twist()
  dists_around = minDistAround() 
  if dists_around[Direction.FRONT.value] <= 2.2 * OBSTACLE_THRESHOLD:
    vel_msg.angular.z = NORMAL_ANGULAR_VEL 
  else:
    vel_msg.linear.x = NORMAL_LINEAR_VEL
    cur_state = States.MoveObstacle 
    print('Start to move along the obstacle!')

  vel_pub.publish(vel_msg)
  

def stateMoveObstacle():
  global init_x, init_y, cur_x, cur_y, goal_x, goal_y
  global obstacle_entry_goal_dist, obstacle_entry_pt, obstacle_pts_sample_cnt
  global vel_pub, cur_state

  flag_same_line = False
  if threePointsSameLine((init_x, init_y), (cur_x, cur_y), (goal_x, goal_y)):
    flag_same_line = True

    if obstacle_pts_sample_cnt <= 0:
      cur_entry_dist = math.sqrt((obstacle_entry_pt[0] - cur_x) ** 2 + (obstacle_entry_pt[1] - cur_y) ** 2)
      if math.fabs(cur_entry_dist) < LINEAR_DIST_THRESHOLD:
        print('Fail! Meet the obstacle entry again: cur-entry-distance {}'.format(cur_entry_dist))
        cur_state = States.Fail
        return
      else:
        cur_goal_dist = math.sqrt((goal_x - cur_x) ** 2 + (goal_y - cur_y) ** 2)
        if cur_goal_dist < obstacle_entry_goal_dist:
          print('Distance reduced! Prepare to move out! cur-goal-distance {}, entry-goal-distance {}'.format(cur_goal_dist, obstacle_entry_goal_dist))
          cur_state = States.FixYaw 
          return

  # Conservative way to identify the first time to leave the M-line and entry point
  if not flag_same_line and obstacle_pts_sample_cnt > 0:
    # if ptDistToLine((init_x, init_y), (cur_x, cur_y), (goal_x, goal_y)) > 3.0 * SAME_LINE_DIST_THRESHOLD:
    cur_entry_dist = math.sqrt((obstacle_entry_pt[0] - cur_x) ** 2 + (obstacle_entry_pt[1] - cur_y) ** 2)
    if math.fabs(cur_entry_dist) > 2.0 * LINEAR_DIST_THRESHOLD:
      print('First time to leave the M-line!')
      obstacle_pts_sample_cnt = 0

  vel_msg = Twist()
  dists_around = minDistAround() 
  if dists_around[Direction.FRONT.value] <= 2.2 * OBSTACLE_THRESHOLD:
    vel_msg.angular.z = NORMAL_ANGULAR_VEL
  else:
    if dists_around[Direction.FRONTRIGHT.value] > 1.5 * OBSTACLE_THRESHOLD:
      vel_msg.linear.x = 0.5 * NORMAL_LINEAR_VEL
      vel_msg.angular.z = -NORMAL_ANGULAR_VEL  
    else: 
      vel_msg.linear.x = 0.5 * NORMAL_LINEAR_VEL 

  vel_pub.publish(vel_msg)
 

def goalPosCallback(pose):
  global cur_state, goal_x, goal_y, init_x, init_y
  if cur_state == States.Done:
    cur_state = States.FixYaw
    init_x = cur_x
    init_y = cur_y 
    goal_x = pose.pose.position.x
    goal_y = pose.pose.position.y

def goalPosListener():
  sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalPosCallback)

def odomCallback(odom):
  global cur_x, cur_y, cur_th 

  position, orientation = odom.pose.pose.position, odom.pose.pose.orientation
  cur_x, cur_y = position.x, position.y

  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [orientation.x, orientation.y, orientation.z, orientation.w] 
  )
  cur_th = yaw
 
def odomListener():
  sub = rospy.Subscriber('/odom', Odometry, odomCallback) 

def laserCallback(laser_scan):
  global laser_each_angle, laser_ranges, running
  laser_each_angle = laser_scan.angle_increment 
  laser_ranges = laser_scan.ranges

def laserListener():
  rospy.Subscriber('/scan', LaserScan, laserCallback)
  
if __name__=="__main__":
  rospy.init_node('hw3_navigation', anonymous=True)

  goalPosListener()
  odomListener()
  laserListener()

  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  state_print_counter = 1
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    if state_print_counter % 30000 == 0:
      state_print_counter = 1
      # print(state_print_counter)
      # print("- Current state: {}.".format(cur_state))
      # print("- Current pos: {}, {}, {}, goal pos: {},{}".format(cur_x, cur_y, cur_th, goal_x, goal_y))
    else:
      state_print_counter += 1

    if cur_state == States.FixYaw:
      stateFixYaw()
    elif cur_state == States.MoveStraight:
      stateMoveStraight()
    elif cur_state == States.TouchObstacle:
      stateTouchObstacle()
    elif cur_state == States.MoveObstacle:
      stateMoveObstacle()
    elif cur_state == States.Done:
      vel_msg = Twist()
      vel_msg.angular.z, vel_msg.linear.x = 0.0, 0.0
      vel_pub.publish(vel_msg)
     
      if send_msg_to_service:
        send_msg_to_service = False
        send_status = sendMsgToService()
        if send_status:
          print('Message has been sent to the service!')

      # print('Arrive at the target!')
    elif cur_state == States.Fail:
      vel_msg = Twist()
      vel_msg.angular.z, vel_msg.linear.x = 0.0, 0.0
      vel_pub.publish(vel_msg)
      cur_state = States.Done
      # print('Fail to reach the target!')
    else: 
      print('Undefined state! Stop the robot now!') 
      vel_msg = Twist()
      vel_msg.angular.z, vel_msg.linear.x = 0.0, 0.0
      vel_pub.publish(vel_msg)
  rate.sleep()
