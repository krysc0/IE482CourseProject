#!/usr/bin/env python3
import sys
import rospy
import copy
from math import pi, inf
import moveit_commander
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
#-------Message types----------------
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose

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
		self.gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
	
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
		while not rospy.is_shutdown():
			self.arm_go_to_joint_state(0,0,0,0)
			rospy.sleep(3)
			#lower arm
			self.arm_go_to_position_goal(0.15,0,0.15)
			rospy.sleep(1)
			#rotate wrist
			self.arm_go_to_joint_state(inf,inf,inf,-1*pi/12)
			#lower arm more
			self.arm_go_to_position_goal(0.25,0,0.10)
			rospy.sleep(2)	
			#open gripper; left .036, right -.036
			self.gripper_go_to_joint_state(0.03,-0.03)
			rospy.sleep(2)
			#move arm lower
			plan, fraction = self.plan_cartesian_path(inf,-0.02)
			self.execute_plan(plan)
			rospy.sleep(2)
			#close gripper, left .02, right -.02
			self.gripper_go_to_joint_state(0.017,-0.017)
			rospy.sleep(2)
			#raise arm
			plan, fraction = self.plan_cartesian_path(0,-0.02)
			self.execute_plan(plan)
			rospy.sleep(2)
			#go to position goal
			self.arm_go_to_position_goal(0.25,0,0.20)
			#rotate waist pi/2
			self.arm_go_to_joint_state(pi/2,inf,inf,inf)
			rospy.sleep(2)
			#lower arm
			plan, fraction = self.plan_cartesian_path(0,-0.02)
			self.execute_plan(plan)
			rospy.sleep(2)
			#open gripper, left .036, right -.036
			self.gripper_go_to_joint_state(0.036,-0.036)
			rospy.sleep(3)
			self.group.clear_pose_target(end_effector_link=self.eef_link)

#-------------------------------Planning to a Joint Goal-----------------------------
	def arm_go_to_joint_state(self,waist,shoulder,elbow,wrist_angle):
		current_joints = self.group.get_current_joint_values()
		print("============ Printing Current Arm Joints State: \n" + str(current_joints))
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

		print("============ Printing Arm Joint Goal: \n" + str(joint_goal))
		self.group.set_joint_value_target(joint_goal)

		self.group.go(wait=True)
		self.group.stop()
		
		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def gripper_go_to_joint_state(self,left,right):
		current_joints = self.gripper.get_current_joint_values()
		print("============ Printing Current Gripper Joint State: \n" + str(current_joints))
		#if function is called with infinity as value then current join value remains unchanged
		joint_goal = current_joints
		if left != inf:
			joint_goal[0] = left
		if right != inf:
			joint_goal[1] = right

		print("============ Printing Gripper Joint Goal: \n" + str(joint_goal))
		self.gripper.set_joint_value_target(joint_goal)

		self.gripper.go(wait=True)
		self.gripper.stop()
		
		current_joints = self.gripper.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)
	
#------------Plan a motion for this group to a desired pose for the end-effector-----------------
	def arm_go_to_position_goal(self,x,y,z):
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
		self.group.set_position_target(position_goal, end_effector_link=self.eef_link)

		plan = self.group.go(wait=True)

		self.group.stop()

		current_pose = self.group.get_current_pose().pose

		return plan

#---You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through-----
	def plan_cartesian_path(self, x_dir, z_dir):
		waypoints = []

		if z_dir != inf:
			wpose = self.group.get_current_pose().pose
			wpose.position.z += z_dir * 0.01
			waypoints.append(copy.deepcopy(wpose))

		if x_dir != inf:
			wpose.position.x += x_dir * 0.01
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
		print(" Pick and Place Node Terminated")	
