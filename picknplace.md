### 1. Install Interbotix Packages
Copy the following code into a new terminal
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh
```
### 2. Launch Robot and World
Open a new terminal, copy and paste the line below
```
roslaunch warehousebot custom.launch robot_model:=px100 use_gazebo:=true dof:=4
```
In a new terminal, copy and paste the lines below
```
rosservice call /gazebo/unpause_physics
rosservice call gazebo/get_world_properties
```
### 3. Run Python Node to Create Conveyor Belt

### 4. Run Python Node to Perform Pick and Place