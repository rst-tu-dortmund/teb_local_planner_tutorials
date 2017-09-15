#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from costmap_converter.msg import ObstacleArrayMsg
from geometry_msgs.msg import PolygonStamped, Point32, Twist
import numpy as np
import matplotlib.pyplot as plotter

def twist_callback(data):
  global trajectory_gt
  global start_t

  if (start_t == 0):
    start_t = rospy.Time.now() # cmd_vel doesn't provide any time information, therefore measure it

  point = TrajectoryPointMsg()
  point.time_from_start = (rospy.Time.now()-start_t)
  point.velocity.linear.x = data.linear.x
  point.velocity.linear.y = data.linear.y
  point.velocity.angular.z = data.angular.z

  trajectory_gt.append(point)


def obstacleArrayMsg_callback(data):
  global trajectory_est
  global start_t

  if (start_t == 0):
    start_t = rospy.Time.now()
  
  point = TrajectoryPointMsg()
  point.time_from_start = (rospy.Time.now()-start_t)
  if(len(data.obstacles) > 0):
    point.velocity.linear.x = data.obstacles[0].velocities.twist.linear.x
    point.velocity.linear.y = data.obstacles[0].velocities.twist.linear.y
    point.velocity.angular.z = data.obstacles[0].velocities.twist.angular.z

    trajectory_est.append(point)
    


def plot_velocity_profiles(fig, ax_vx, ax_vy, t_gt, v_x_gt, v_y_gt, omega_gt, t_est, v_x_est, v_y_est, omega_est):
  ax_vx.cla()
  ax_vx.grid()
  ax_vx.set_ylabel('Trans. velocity (x) [m/s]')
  ax_vx.plot(t_gt, v_x_gt, '-bx', label='Ground Truth')
  ax_vx.plot(t_est, v_x_est, '-rx', label='Estimate')
  ax_vx.legend(loc='upper left')
  ax_vy.cla()
  ax_vy.grid()
  ax_vy.set_ylabel('Trans. velocity (y) [m/s]')
  ax_vy.plot(t_gt, v_y_gt, '-bx', label='Ground Truth')
  ax_vy.plot(t_est, v_y_est, '-rx', label='Estimate')
  ax_vy.set_xlabel('Time [s]')
  fig.canvas.draw()

  
def velocity_plotter():
  global trajectory_gt
  global trajectory_est
  global start_t

  rospy.init_node("visualize_obstacle_velocity_profile", anonymous=True)
  
  topic_name_ground_truth_vel = "cmd_vel"
  rospy.Subscriber(topic_name_ground_truth_vel, Twist, twist_callback, queue_size = 1)
  topic_name_estimated_vel = "/standalone_converter/costmap_obstacles"
  rospy.Subscriber(topic_name_estimated_vel, ObstacleArrayMsg, obstacleArrayMsg_callback, queue_size = 1)

  rospy.loginfo("Visualizing ground truth velocity profile published on '%s'.", topic_name_ground_truth_vel)
  rospy.loginfo("Visualizing estimated velocity profile published on '%s'.", topic_name_estimated_vel)
    
  # two subplots sharing the same t axis
  fig, (ax_vx, ax_vy) = plotter.subplots(2, sharex=True)
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    t_gt = []
    v_x_gt = []
    v_y_gt = []
    omega_gt = []

    t_est = []
    v_x_est = []
    v_y_est = []
    omega_est = []
    
    for point in trajectory_gt:
      t_gt.append(point.time_from_start.to_sec())
      v_x_gt.append(point.velocity.linear.x)
      v_y_gt.append(point.velocity.linear.y)
      omega_gt.append(point.velocity.angular.z)

    for point in trajectory_est:
      t_est.append(point.time_from_start.to_sec())
      v_x_est.append(point.velocity.linear.x)
      v_y_est.append(point.velocity.linear.y)
      omega_est.append(point.velocity.angular.z)

          
    plot_velocity_profiles(fig, ax_vx, ax_vy, np.asarray(t_gt), np.asarray(v_x_gt), np.asarray(v_y_gt), np.asarray(omega_gt), 
                            np.asarray(t_est), np.asarray(v_x_est), np.asarray(v_y_est), np.asarray(omega_est))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory_gt = []
    trajectory_est = []
    start_t = 0
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass
#  finally:
#    plotter.savefig('/home/albers/Desktop/velocity_plot.pdf', bbox_inches='tight')

