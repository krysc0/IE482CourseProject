#!/usr/bin/env python3
import rospy
import math
import random
#-------------Message types----------------
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelStates
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetModelStates
#------------------------------------------
class conveyor():
	def __init__(self):
		#Initialize node
		rospy.init_node('move_object', anonymous=True)
		#Wait for services
		rospy.wait_for_service('/gazebo/get_world_properties')
		rospy.wait_for_service('/gazebo/get_model_state')
		rospy.wait_for_service('/gazebo/set_model_state')
		print("Services Ready")
		#Get name of the models in the Gazebo world
		self.msg= rospy.wait_for_service('/gazebo/get_world_properties', GetWorldProperties, timeout=None)
		self.names = GetWorldProperties.model_names
		print(self.names)
		self.models={}
		#Receive initial location of each model
		self.msg= rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None)
		self.model_intial_pos(self.msg)
		#Start server/client relationship with gazebo
		self.call_get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
		self.call_set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
		#Define Parameters
		self.MIN_SPEED = 0.05
		self.MAX_SPEED = 0.2
		self.convey_start=[2,0.25,0.012]

		rospy.Rate = 0.2
		for model_name in self.names:
			if model_name not in ["ground_plane", "px100","conveyor_belt_0","euro_pallet"]
				#Once model is at the height of the place location, we use a different model on the conveyor
				while where_is(model_name)[2]< 0.1 :
					for command in [self.moveto(model_name),self.conveyor(model_name)]:					
						pubCmd = False							
						if (rospy.Time.now() >= nextTime):
							# It's now time to publish a command
							pubCmd = True
							(cmd,nextTime) = command
							
						if (pubCmd):
							# Issue the service request to move the model:
							myResponse = self.call_set_model_state(cmd)
							# print myResponse
				
							# Otherwise, print an error message:
							if (not myResponse.success):
								print("Error")

	#Return dictionary with format below of initial position of each model   
	#{"model name": [x,y,z] }
	def model_initial_pos(self,msg):
		for m,mod in enumerate(msg.pose):
			if self.names[m] not in ["ground_plane","px100"]:
				self.models[names[m]] = [mod.position.x,mod.position.y,mod.position.z]

	def setStationary(self, model_name):
		# Where is the model right now?
		(x, y, z) = self.whereAmI(model_name)
		# How long do we want this model to stay still?
		duration = 1		
		# At what time should we stop executing this command?
		nextTime = rospy.Time.now() + rospy.Duration(duration)
		# Initialize an empty 'ModelState' message.
		# We have to send this type of message to the service to get the model to move.
		cmd = ModelState()
		# Add some details to the message:
		cmd.model_name 		= model_name
		cmd.reference_frame	= 'world'
		cmd.pose.position.x	= x
		cmd.pose.position.y	= y
		cmd.pose.position.z	= z
	
		return (cmd, nextTime)

	#Find model at any point in time 
	def where_is(self,model_name):
		# 1) Call Gazebo Service:
		modelLoc = self.call_get_model_state(model_name, '')
		# 2) Grab just the info we need:
		[x, y, z] = [modelLoc.pose.position.x, modelLoc.pose.position.y, modelLoc.pose.position.z]
		# print x, y, z
		print(" %s is at (%g,%g,%g%)" %(model_name,x,y,z))
		# 3) Return this info:
		return [x, y, z]
		
	#Move model to the beginning of the conveyor belt
	def moveto(self, model_name):
		# Where is the model right now?
		[xNow, yNow, zNow] = self.whereAmI(model_name)
		# Where do you want it to go?
		[xGoal, yGoal, zGoal] = self.convey_start
		# How far is it from the current location to the goal location?
		dist = math.sqrt( (xNow-xGoal)**2 + (yNow-yGoal)**2 + (zNow-zGoal)**2 )
		# How fast should the model move?
		speed = random.uniform(MIN_SPEED, MAX_SPEED)
		# How long will it take to get to the goal location?
		duration = dist/speed		
		# At what time should we stop executing this command?
		nextTime = rospy.Time.now() + rospy.Duration(duration)
		# Initialize an empty 'ModelState' message.
		# We have to send this type of message to the service to get the model to move.
		cmd = ModelState()

		# Add some details to the message:
		cmd.model_name 		= model_name
		cmd.reference_frame	= 'world'
		cmd.pose.position.x	= xNow
		cmd.pose.position.y	= yNow
		cmd.pose.position.z	= zNow
		cmd.twist.linear.x	= (xGoal - xNow) / duration
		cmd.twist.linear.y	= (yGoal - yNow) / duration
		cmd.twist.linear.z	= (zGoal - zNow) / duration

		# print cmd
		
		return (cmd, nextTime)
	
	#Move model along conveyor belt
	def conveyor(self, model_name):
		# Where is the model right now?
		[xNow, yNow, zNow] = self.whereAmI(model_name)
		# Where do you want it to go?
		[xGoal, yGoal, zGoal] = (0,0.25,0.012)
		# Straight line in the x direction
		dist = xNow-xGoal
		# How fast should the model move?
		speed = 0.2
		# How long will it take to get to the goal location?
		duration = dist/speed		
		# At what time should we stop executing this command?
		nextTime = rospy.Time.now() + rospy.Duration(duration)
		# Initialize an empty 'ModelState' message.
		# We have to send this type of message to the service to get the model to move.
		cmd = ModelState()

		# Add some details to the message:
		cmd.model_name 		= model_name
		cmd.reference_frame	= 'world'
		cmd.pose.position.x	= xNow
		cmd.pose.position.y	= yNow
		cmd.pose.position.z	= zNow
		cmd.twist.linear.x	= (xGoal - xNow) / duration

		# print cmd	
		return (cmd, nextTime)

if __name__ == "__main__":
	try:
		bot()
	except rospy.ROSInterruptException:
		print("Pick and Place Node Terminated")	
