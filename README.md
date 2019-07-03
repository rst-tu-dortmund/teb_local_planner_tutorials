# teb_local_planner_tutorials
This package contains supplementary material and examples for [teb_local_planner](http://wiki.ros.org/teb_local_planner) tutorials.

The tutorials package mainly contains fully working robot navigation examples in combination with the teb_local_planner.
Currently it provides a differential drive and a carlike robot simulation setup.
In order to depend on as few dependencies as possible, the simulations are performed with [stage_ros](http://wiki.ros.org/stage_ros)
and without any URDF models. However, they are easily extendable and integrable (e.g. Gazebo, URDF models, voxel costmaps, robot hardware nodes, ...).

Refer to the [teb_local_planner](http://wiki.ros.org/teb_local_planner) ROS wiki page for more information.

**Dependencies:**

 * *navigation stack* and *teb_local_planner* package
 * *stage*: `sudo apt-get install ros-kinetic-stage-ros`

