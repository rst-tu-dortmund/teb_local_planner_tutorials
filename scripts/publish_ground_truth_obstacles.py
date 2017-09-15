#!/usr/bin/env python

import rospy, math, random, re
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, PolygonStamped, Quaternion, QuaternionStamped, TwistWithCovariance, Point32
from nav_msgs.msg import Odometry
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

# global variables
obstacles_msg = ObstacleArrayMsg()
obst_pub = rospy.Publisher('/robot_0/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size = 10)

def callback_base_pose_ground_truth(base_pose_ground_truth, obst_id):
  # Search for obstacle id and update fields if found
  i = -1 # -1 is the last element
  for idx, obst in enumerate(obstacles_msg.obstacles):
    if obst.id == obst_id:
      i = idx
   
  if i == -1:
    obstacle_msg = ObstacleMsg()
    obstacle_msg.id = obst_id
    obstacles_msg.obstacles.append(obstacle_msg)

  # HEADER
  obstacles_msg.header.stamp = rospy.Time.now()
  obstacles_msg.header.frame_id = 'map'

  # OBSTACLES
  obstacles_msg.obstacles[i].header.stamp = obstacles_msg.header.stamp
  obstacles_msg.obstacles[i].header.frame_id = obstacles_msg.header.frame_id
  obstacles_msg.obstacles[i].polygon.points = [Point32()] # currently only point obstacles
  obstacles_msg.obstacles[i].polygon.points[0].x = base_pose_ground_truth.pose.pose.position.x
  obstacles_msg.obstacles[i].polygon.points[0].y = base_pose_ground_truth.pose.pose.position.y
  obstacles_msg.obstacles[i].polygon.points[0].z = base_pose_ground_truth.pose.pose.position.z

  # ORIENTATIONS
  yaw = math.atan2(base_pose_ground_truth.twist.twist.linear.y, base_pose_ground_truth.twist.twist.linear.x)
  quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw in radians
  obstacles_msg.obstacles[i].orientation = Quaternion(*quat.tolist())

  # VELOCITIES
  obstacles_msg.obstacles[i].velocities = base_pose_ground_truth.twist


def init_subscribers():
  rospy.init_node("GroundTruthObstacles")

  # Setup a subscriber for each obstacle -> get obstacle ids from published topics
  published_topics, published_types = zip(*rospy.get_published_topics())
  r = re.compile("^\/robot_([1-9][0-9]*)\/base_pose_ground_truth$")
  bpgt_topics = filter(r.match, published_topics)

  for topic in bpgt_topics:
    match = r.search(topic)
    obst_id = int(match.group(1))
    sub = rospy.Subscriber(topic, Odometry, callback_base_pose_ground_truth, obst_id)

  r = rospy.Rate(10)

  while not rospy.is_shutdown():
    # Publish Obstacle_msg
    obst_pub.publish(obstacles_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    init_subscribers()
  except rospy.ROSInterruptException:
    pass

