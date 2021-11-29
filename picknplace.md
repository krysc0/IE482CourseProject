# Pick and Place using Interbotic Pincher x 100
## 1. Clone github
## 2. Launch Robot and World
Open a terminal
```
roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=px100 dof:=4 use_trajectory_controllers:=true use_rviz:=true
```
In a new terminal
```
rosservice call /gazebo/unpause_physics
rosservice call gazebo/get_world_properties
```