#!/usr/bin/env python3
import rospy
import numpy as np

#-------Joints------------
#--arm--	--gripper--
#wrist		left finger	
#elbow		right finger	
#shoulder
#waist	 		

#-------Message types----------------
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal


class bot():
	def __init__(self):
		#initialize node
		rospy.init_node('pick_object', anonymous=True)

		#Receive one message of the model states (assuming that models remain stationary)
		self.msg= rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None)

		#Call function to save name and location of each model in Gazebo world
		self.model_state_callback(self.msg)

		#Publishing to topics for each joint group		
		self.direct_arm = rospy.Publisher('/px100/arm_controller/follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal,queue_size=10)
		self.direct_gripper = rospy.Publisher('/px100/gripper_controller/follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal,queue_size=10)
	
	# objective = FollowJointTrajectoryActionGoal()
	#objective.goal.trajectory.joint_names
	# objective.goal.trajectory.points (1 for each of 5 joints) .positions
	#def pick(self):
		#Given self.models, move robot to that position, pick up the object
	#def place(self):

	#return dictionary with format below, excluding the ground plane and the robotic arm itself 
	#{ "model name": [x,y,z] }
	def model_state_callback(self,msg):
		self.models={}
		names=[]
		print("Models in Gazebo world are")
		for model_name in msg.name:
			names.append(model_name)
			print(model_name)
		for m,mod in enumerate(msg.pose):
			if names[m] not in ["ground_plane","px100"]:
				self.models[names[m]] = [mod.position.x,mod.position.y,mod.position.z]


if __name__ == "__main__":
	try:
		bot()
	except rospy.ROSInterruptException:
		print("Pick and Place Node Terminated")	
