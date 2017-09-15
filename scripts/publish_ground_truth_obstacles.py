#!/usr/bin/env python

import rospy, math, random, re
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, PolygonStamped, Quaternion, QuaternionStamped, TwistWithCovariance, Point32
from nav_msgs.msg import Odometry
from teb_local_planner.msg import ObstacleMsg

# global variables
Obstacle_msg = ObstacleMsg()
obst_pub = rospy.Publisher('/robot_0/move_base/TebLocalPlannerROS/obstacles', ObstacleMsg, queue_size = 10)

def callback_base_pose_ground_truth(base_pose_ground_truth, obst_id):
  # Search for obstacle id and update fields, if found
  # try to find the current id
  try:
    i = Obstacle_msg.ids.index(obst_id)
  # Obstacle with current id not published so far
  except ValueError:
    i = -1 # -1 is the last element
    Obstacle_msg.ids.append(obst_id)
    Obstacle_msg.obstacles.append(PolygonStamped())
    Obstacle_msg.orientations.append(QuaternionStamped())
    Obstacle_msg.velocities.append(TwistWithCovariance())

  # HEADER
  Obstacle_msg.header.stamp = rospy.Time.now()
  Obstacle_msg.header.frame_id = 'map' # TODO? Seems to works this way..

  # OBSTACLES
  Obstacle_msg.obstacles[i].header.stamp = Obstacle_msg.header.stamp
  Obstacle_msg.obstacles[i].header.frame_id = Obstacle_msg.header.frame_id
  Obstacle_msg.obstacles[i].polygon.points = [Point32()] # currently only point obstacles
  Obstacle_msg.obstacles[i].polygon.points[0].x = base_pose_ground_truth.pose.pose.position.x
  Obstacle_msg.obstacles[i].polygon.points[0].y = base_pose_ground_truth.pose.pose.position.y
  Obstacle_msg.obstacles[i].polygon.points[0].z = base_pose_ground_truth.pose.pose.position.z

  # ORIENTATIONS
  Obstacle_msg.orientations[i].header.stamp = Obstacle_msg.header.stamp
  Obstacle_msg.orientations[i].header.frame_id = Obstacle_msg.header.frame_id
  yaw = math.atan2(base_pose_ground_truth.twist.twist.linear.y, base_pose_ground_truth.twist.twist.linear.x)
  quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw in radians
  Obstacle_msg.orientations[i].quaternion = Quaternion(*quat.tolist())

  # VELOCITIES
  Obstacle_msg.velocities[i] = base_pose_ground_truth.twist


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
    obst_pub.publish(Obstacle_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    init_subscribers()
  except rospy.ROSInterruptException:
    pass

