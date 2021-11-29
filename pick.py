#!/usr/bin/env python3
import rospy
import numpy as np

#-------Joints------------
#wrist			#waist
#elbow			#left finger
#shoulder		#right finger

#-------Message types----------------
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

class bot():
	def __init__(self):
		#initialize node
		rospy.init_node('pick_object', anonymous=True)

		#Receive one message of the model states (assuming that models remain stationary)
		self.msg= rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None)

		#Call function to save name and location of each model in Gazebo world
		self.model_state_callback(self.msg)

		#Publishing to command topic for each joint
		self.control_elbow = rospy.Publisher('/px100/elbow_controller/command',Float64,queue_size=1)
		self.control_shoulder = rospy.Publisher('/px100/shoulder_controller/command',Float64,queue_size=1)
		self.control_left_finger = rospy.Publisher('/px100/left_finger_controller/command',Float64,queue_size=1)
		self.control_right_finger = rospy.Publisher('/px100/right_finger_controller/command',Float64,queue_size=1)
		self.control_waist = rospy.Publisher('/px100/waist_controller/command',Float64,queue_size=1)
		self.control_wrist_controller = rospy.Publisher('/px100/wrist_angle_controller/command',Float64,queue_size=1)
		
		self.r = Float64()
		self.r.data = 0.07
		rate=rospy.Rate(10)
		#Publish relative to home pose
		while not rospy.is_shutdown():
			self.control_waist.publish(self.r)
			rate.sleep()

	
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
