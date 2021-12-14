#!/usr/bin/env python3
import sys
import rospy
import copy
from math import pi, inf
import moveit_commander
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
#-------Message types----------------
from moveit_msgs.msg import Grasp,DisplayTrajectory, PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import PoseStamped, Pose, Point
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

#-------Joints------------
#--arm--	--gripper--
#wrist		left finger	
#elbow		right finger	
#shoulder
#waist	

#---------------------Parameters--------------------
#Where gets pickedby the robotic arm
pick_pos=[0.15,0.15,0.05]
#Where robotic arm places the model
place_pos_min=[0.3,-0.17,0.07]
place_pos_max=[-0.3,-0.17,0.07]

# Tolerances for achieving a joint goal, joint, position, orientation = (0.0001, 0.0001, 0.001)
# End effector Link = ee_gripper_link
# arm_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'ee_arm', 'gripper_bar', 'ee_bar', 'ee_gripper']

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class bot():
	def __init__(self):
		#intialize moveit commander
		moveit_commander.roscpp_initialize(sys.argv)
		#initialize node
		rospy.init_node('pick_object', anonymous=True)

		#Provides robot's kinematic model and the robot's current joint states
		self.robot = moveit_commander.RobotCommander(robot_description="px100/robot_description")
		#provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
		self.scene = moveit_commander.PlanningSceneInterface()

		group_name="interbotix_arm"
		self.group = moveit_commander.MoveGroupCommander(group_name)
	
		#Publish topic to display trajectory in Rviz 
		self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",DisplayTrajectory, queue_size=20)
		
		#Get Parameters
		self.planning_frame = self.group.get_planning_frame()
		self.eef_link = self.group.get_end_effector_link()
		self.tolerances = self.group.get_goal_tolerance()
		self.group.set_goal_position_tolerance(0.01)
		self.group.set_goal_orientation_tolerance(0.01)
		self.group.set_goal_joint_tolerance(0.01)
		self.group.set_planning_time(10)	
		
		
		#------------------------------------Pick and Place----------------------------------------------------------
		#Get Pose Goal / Joint Goal
		#set_pose_target
		#set_joint_target
		# The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
		#self.group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		#self.group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		#self.group.clear_pose_targets()

		self.go_to_position_goal(0.15,0,0.1)
		self.go_to_joint_state(inf,inf,inf,pi/4)


#-------------------------------Planning to a Joint Goal-----------------------------
	def go_to_joint_state(self,waist,shoulder,elbow,wrist_angle):
		current_joints = self.group.get_current_joint_values()
		print("============ Printing Current Joint State: " + str(current_joints))
		#if function is called with infinity as value then current join value remains unchanged
		joint_goal = current_joints
		if waist != inf:
			joint_goal[0] = waist
		if shoulder != inf:
			joint_goal[1] = shoulder
		if elbow != inf:
			joint_goal[2] = elbow
		if wrist_angle != inf:
			joint_goal[3] = wrist_angle

		print("============ Printing Joint Goal: " + str(joint_goal))
		self.group.set_joint_value_target(joint_goal)

		self.group.go(wait=True)
		self.group.stop()
		
		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)
	
#------------Plan a motion for this group to a desired pose for the end-effector-----------------
	def go_to_position_goal(self,x,y,z):
		current_pose = self.group.get_current_pose().pose
		print("============ Printing Current Pose: " + str(current_pose) + "==============")

		position_goal = [0,0,0]
		if x != inf:
			position_goal[0] = x
		if y != inf:
			position_goal[1] = y
		if z != inf:
			position_goal[2] = z

		print("============ Printing Position Goal: " + str(position_goal) + "==============")
		self.group.set_position_target(position_goal, self.eef_link)

		plan = self.group.go(wait=True)

		self.group.stop()

		current_pose = self.group.get_current_pose().pose

		return plan

#---You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through-----
	def plan_cartesian_path(self, x_dir, z_dir):
		waypoints = []

		wpose = self.group.get_current_pose().pose
		wpose.position.z += z_dir * 0.01  # First move up (z)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += x_dir * 0.01  # Second move forward in (x)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = self.group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.0)         # jump_threshold

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

	def execute_plan(self, plan):
		self.group.execute(plan, wait=True)

if __name__ == "__main__":
	try:
		bot()
	except rospy.ROSInterruptException:
		print("Pick and Place Node Terminated")	
