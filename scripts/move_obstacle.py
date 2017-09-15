#!/usr/bin/env python

import rospy, math, random
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

# Parameters
vel_max = 1.0
vel_min = 0.1

# Define parameters, if parameter server provides no definitions
# Lower bound of the obstacle movement in y-direction
if rospy.has_param("pos_lb"):
  pos_lb = rospy.get_param("pos_lb")
else:
  pos_lb = 0.5

# Upper bound of the obstacle movement in y-direction
if rospy.has_param("pos_ub"):
  pos_ub = rospy.get_param("pos_ub")
else:
  pos_ub = 5.5

# Define global Twist message which is modified in the callback_base_pose_ground_truth and published in the main loop
Twist_msg = Twist()


# This function manages turnarounds and new (possibly random) velocities of the object
def callback_base_pose_ground_truth(base_pose_ground_truth):
  # Define random velocity if no specific velocity is defined in the parameter server
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  else:
  	vel_y = random.uniform(vel_min, vel_max)

  # Turn in y-direction
  if base_pose_ground_truth.pose.pose.position.y >= pos_ub:
    Twist_msg.linear.y = -vel_y
  if base_pose_ground_truth.pose.pose.position.y <= pos_lb:
    Twist_msg.linear.y = vel_y


# This function initializes the mover node and publishes continously a Twist message
def move_object():
  rospy.init_node("Mover")
  r = rospy.Rate(10)
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('base_pose_ground_truth', Odometry, callback_base_pose_ground_truth)
  start_t = rospy.get_time()

  # initialize movement with random direction
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  else:
  	vel_y = random.uniform(vel_min, vel_max)
  Twist_msg.linear.y = float(random.choice(['-1', '1'])) * vel_y

  # publish movement command continuously
  while not rospy.is_shutdown():
    pub.publish(Twist_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    move_object()
  except rospy.ROSInterruptException:
    pass