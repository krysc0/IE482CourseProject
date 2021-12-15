# Pick and Place using Interbotix Robotic Arm

Project Name: 'warehousebot'  

Team Members:
- Krystal Coleman, kc64739@buffalo.edu
- Omar-Ibne Shahid, omaribne@buffalo.edu

---

## Project Description

*In this section, describe what your project does. This should be descriptive.  Someone from next year's class should be able to fully understand the aims and scope of your project. I highly recommend using pictures to help explain things.  Maybe even post a YouTube video showing your code in action.*

*NOTE:  This is not a proposal.  This is a final report describing your actual completed project.*

### Contributions

*In this subsection, I want to know what is new/unique/interesting about your project.*

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
*Provide detailed step-by-step instructions to run your code.*

*NOTE 1:  At this point, the user should have already installed the necessary code.  This section should simply describe the steps for RUNNING your project.*  

*NOTE 2:  If you're generating mazes, for example, the task of GENERATING a new maze would go here.*

---

## Additional Section - Guideline to play with the basic functions of the robot

**NB:This is totally sperated from previous sections, one can run only this section of tutorial to see the basic functions of the robot, don't need to run previous sections.**

### 1. Downloads and Installation
```
cd ~
git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
```

```
cd ~
cd interbotix_ros_manipulators/interbotix_ros_xsarms
```

Now copy following folders to catkin_ws/src:
interbotix_xsarm_control, interbotix_xsarm_descriptions, interbotix_xsarm_gazebo, interbotix_xsarm_perception, interbotix_xsarm_ros_control

Now run the following command to build all necessary packages:

```
catkin clean -y
```
```
catkin_make
```



### 2. Running commands for movement
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
	<TD>50% (We can move our robot in gazebo world, but couldn't extract the position of the objects to reach, therefore, it's reamin undone.)</TD>
</TR>
<TR>
	<TD>Robotic arm can move to specific location and release object</TD>
	<TD>50% (Robotic arm can move to a speicific location and function its gripper, but as we are unable to pick up object, so this task is also incomplete.)</TD>
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

* It was our very first experience to work with moveit, we did it from scratch. We did the interface of moveit with gazebo

* We explored to work with industrial robotic arm, which we never did before

* We figured out how to work with moving objects

* We did all the coding by exploring the datasheet of the robotic arm, therefore, we have gained the confidence that we can work with something totally new

---

## Future Work

Prime challenge we have faced in this project is to find a proper robotic arm for ROS Noetic version as most of the available robotic arm available online are compatible for previous ROS versions and also built-in libraries are dveloped with cpp rather than python. But whoever work in future, can use our basic functioning code and installation guide to setup the environment for the robot. As we left some unfinished business of pick and place object in gazebo world, so future students can do the synchronization of our gazebo world of conveyor belt with robotic arm movements which will enable them to complete the pick and place taks successfully. In next stage, they can try to implement their code on real physical robot to see how their algorithm works. 

---

## References/Resources

*What resources did you use to help finish this project?*
- `https://github.com/Interbotix` Specifically the interbotix_ros_manipulators and interbotix_ros_toolboxes repositories
- `https://ros-planning.github.io/moveit_tutorials/` Specifically the Move Group Python Interface 
