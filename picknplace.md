### 1. Install Moveit 
```
sudo apt install ros-noetic-moveit
```
### 2. Install Packages for Interbotix Robots
Copy the following code into a new terminal
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh
```
When prompted to:
* `Install the Perception Pipeline (includes RealSense and AprilTag packages)?`
* `Install the MATLAB API (requries that you have MATLAB installed on your system)?`

Respond with `n` to both
### 3. Copy Interbotix Packages into catkin_ws
```
cd ~
cp -R ~/interbotix_ws/* ~/catkin_ws/src
```
### 4. Clone Warehousebot Github 
```
cd ~/catkin_ws/src
catkin_create_pkg warehousebot
git clone https://github.com/krysc0/IE482CourseProject.git
cp -R ~/catkin_ws/src/IE482CourseProject/code/warehousebot/* ~/catkin_ws/src/warehousebot/
cd ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
```
### 5. Build the code
```
cd ~/catkin_ws
catkin_make

```
### 6. Launch Robot in Gazebo World and Rviz
Open a new terminal, copy and paste the line below
```
cd ~/catkin_ws
roslaunch warehousebot custom.launch robot_model:=px100 use_gazebo:=true dof:=4
```
In a new terminal, copy and paste the lines below
```
rosservice call /gazebo/unpause_physics
```
### 7. Run Python Node to Create Conveyor Belt
```
roscd warehouesbot/scripts
chmod +x conveyor.py
rosrun warehousebot conveyor.py
```
### 8. Run Python Node to Perform Pick and Place
```
roscd warehouesbot/scripts
chmod +x pick.py
ROS_NAMESPACE=px100 rosrun warehousebot pick.py 
```
