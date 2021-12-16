# Pick and Place using Interbotix Robotic Arm

Project Name: 'warehousebot'  

Team Members:
- Krystal Coleman, kc64739@buffalo.edu
- Omar-Ibne Shahid, omaribne@buffalo.edu

---

## Project Description

Our project is about simulating a robotic arm in gazebo performing a pick and place task. We have used an nterbotix robotic arm and a customized gazebo world. We used moveit platform to interact with the robotic arm.

### Contributions
One of the main contribution of our project is there is no sufficient resource for beginners to work with robotic arm in ROS Noetic. Our project will provide a head start to those who want to work with industrial automation in ROS Noetic.
---

## Installation Instructions
List of Prerequisite Software:
- Moveit 
- Interbotix XS Manipulators
- Gazebo

### 1. Install Moveit from Source
#### a. Make sure you have the latest versions of packages installed
```
rosdep update
sudo apt update
sudo apt dist-upgrade
```
#### b. Source installation requires wstool and catkin_tools
```
sudo apt install python3-catkin-tools
sudo apt install python3-wstool
```
#### c. Create a new workspace and download moveit
```
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove  moveit_tutorials  # this is cloned in the next section
wstool update -t .
```
#### d. Build your caktin workspace
```
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```
#### e. Configure catkin workspace (WARNING: This takes around 75 Minutes)
```
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
#### f. Source the catkin workspace
```
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```
### 2. Install Packages for Interbotix Robots
#### a. Installation
Note. This installation may take up to 15 minutes
Install curl
```
sudo apt install curl
```
```
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh
```
When prompted to:
* `Install the Perception Pipeline (includes RealSense and AprilTag packages)?`
* `Install the MATLAB API (requries that you have MATLAB installed on your system)?`

Respond with `n` to both
#### b. Edit the Kinematics Configuration File
```
pico interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit/config/kinematics.yaml
```
Find and replace `position_only_ik: false`
with `position_only_ik: true`

### 3. Get Warehousebot files 
#### a. Clone github
```
cd ~/catkin_ws/src
catkin_create_pkg warehousebot
cd warehousebot 
mkdir scripts
cd ~/catkin_ws/src
git clone https://github.com/krysc0/IE482CourseProject.git
cp -R ~/catkin_ws/src/IE482CourseProject/code/warehousebot/scripts/* ~/catkin_ws/src/warehousebot/scripts
```
#### b. Build the catkin workspace
```
cd ~/catkin_ws/
rm -r build
rm -r devel
catkin build
```
#### c. Source the workspace
```
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```
---

## Running the Code
### 1. Launch Robot in Gazebo World and Rviz
Open a new terminal, copy and paste the line below
```
source ~/catkin_ws/devel/setup.bash
roslaunch warehousebot custom.launch
```
In Terminal 2
```
rosservice call /gazebo/unpause_physics
```
### 2. Run Python Node to Create Conveyor Belt
In Terminal 3
```
source ~/catkin_ws/devel/setup.bash
roscd warehousebot/scripts
chmod +x conveyor.py
rosrun warehousebot conveyor.py
```
### 3. Run Python Node to Perform Pick and Place
In Terminal 4
```
source ~/catkin_ws/devel/setup.bash
roscd warehousebot/scripts
chmod +x pick.py
ROS_NAMESPACE=px100 rosrun warehousebot pick.py 
```
### 4. Shutting Down the Nodes
In the same terminal you launched the pick and place node
press `Ctrl+C` and wait for the processes to end
Then return to the terminal you launched Gazebo in
press `Ctrl-C` and wait for the processes to end

---
## Understanding the Code
The custom launch file used is a modified version of the `xsarm_moveit.launch` file located in the `interbotix_xsarm_moveit package`.Using the paramter "use_gazebo" and a custom gazebo world this launch file, when run, launches rviz and gazebo with the custom world, the robot model, the robot description and the robot's move groups. 



## Additional Section - Guideline to play with the basic functions of the robot
**NB:This is totally sperated from previous sections, one can run only this section of tutorial to see the basic functions of the robot, don't need to run previous sections.**

### 1. Running commands for movement
1. Launching the world
```
roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=wx200 dof:=5 use_position_controllers:=true
```
2. Open another terminal, run the following commands to see what topics you have in this arm.
```
rosservice call /gazebo/unpause_physics
```
```
rostopic list
```
Here, we will use wrist_angle_controller, elbow_controller, waist_controller, left_finger_controller, and right_finger_controller.

3. Move the wrist
```
rostopic pub -1 /wx200/wrist_angle_controller/command std_msgs/Float64 "data: -1.0"
```
4. Move elbow
```
rostopic pub -1 /wx200/elbow_controller/command std_msgs/Float64 "data: 1.0"
```
5. Move waist
```
rostopic pub -1 /wx200/waist_controller/command std_msgs/Float64 "data: 0.7"
```

```
rostopic pub -1 /wx200/waist_controller/command std_msgs/Float64 "data: -0.7"
```
6. Gripper movement
```
rostopic pub -1 /wx200/left_finger_controller/command std_msgs/Float64 "data: -0.037"
```

```
rostopic pub -1 /wx200/left_finger_controller/command std_msgs/Float64 "data: 0.015"
```

```
rostopic pub -1 /wx200/right_finger_controller/command std_msgs/Float64 "data: -0.015"
```

## Measures of Success

<TABLE>
<TR>
	<TH>Measure of Success (from PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>Creation of conveyor belt in Gazebo</TD>
	<TD>100% (It works totally fine.)</TD>
</TR>
<TR>
	<TD>Robotic arm can lift stationary object</TD>
	<TD>75% (We can move our robot in gazebo world, but couldn't extract the position of the objects to reach, therefore, it's reamin undone.)</TD>
</TR>
<TR>
	<TD>Robotic arm can move to specific location and release object</TD>
	<TD>75% (Robotic arm can move to a speicific location and function its gripper, but as we are unable to pick up object, so this task is also incomplete.)</TD>
</TR>
<TR>
	<TD>Robotic arm can pick and place objects that were initially moving</TD>
	<TD>50% (As we have mentioned previously, we couldn't extract the position of the objects to reach, therefore, it's reamin undone.)</TD>
</TR>
<TR>
	<TD>Robot can differentiate between items of different colors</TD>
	<TD>0% (We didn't get any chance to explore this goal)</TD>
</TR>
</TABLE>



## What did you learn from this project?

* How to use the moveit move group python interface
* We explored motions of a robotic arm, which we had never done before
* We figured out how to work with moving objects
* We did all the coding from scratch by exploring the data about the robotic arm, therefore, we have gained the confidence that we can work with something totally new

---

## Future Work

Prime challenge we have faced in this project is to find a proper robotic arm for ROS Noetic version as most of the available robotic arm available online are compatible for previous ROS versions and also built-in libraries are dveloped with cpp rather than python. But in the future our basic functioning code and installation guide can setup the environment for the robot. We left some unfinished business of pick and place object in gazebo world, in the future we'd like to synchronize our gazebo world of a conveyor belt with robotic arm movements which will enable them to complete the pick and place taks successfully. In next stage also, implementation of this code on a real physical robot could be attempted.

---

## References/Resources

*What resources did you use to help finish this project?*
- `https://github.com/Interbotix` Specifically the interbotix_ros_manipulators and interbotix_ros_toolboxes repositories
- `https://ros-planning.github.io/moveit_tutorials/` Specifically the Move Group Python Interface 
