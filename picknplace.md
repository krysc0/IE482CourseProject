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

## Measures of Success

*You have already defined these measures of success (MoS) in your proposal, and updated them after your progress report.  The purpose of this section is to highlight how well you did.  Also, these MoS will be useful in assigning partial credit.*

*The MoS summary should be in table form.  A sample is provided below:*
<TABLE>
<TR>
	<TH>Measure of Success (from your PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>Install PR2 ROS Indigo Package</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Write brain reader software to move the robot</TD>
	<TD>25% (brain reader software detects brain waves, but does not translate to ROS commands.)</TD>
</TR>
</TABLE>

*NOTE 1:  I have your proposals...don't move the goal posts!*

*NOTE 2:  For activities less than 100% complete, you should differentiate between what you completed and what you were unable to complete. I suggest you add details in a bullet list below.* 


---

## What did you learn from this project?

*For example, what concepts from class do you now have a solid understanding of?  What new techniques did you learn?*

*Also, what challenges did you face, and how did you overcome these?  Be specific.*

---

## Future Work

*If a student from next year's class wants to build upon your project, what would you suggest they do?  What suggestions do you have to help get them started (e.g., are there particular Websites they should check out?).*

---

## References/Resources

*What resources did you use to help finish this project?*
- Include links to Websites.  Explain what this Website enabled you to accomplish.
- Include references to particular chapters/pages from the ROS book.  Why was each chapter necessary/helpful?
