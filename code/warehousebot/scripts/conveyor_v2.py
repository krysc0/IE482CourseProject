#!/usr/bin/env python3
import rospy
import random
import math
#-------------Message/Service types----------------
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
#------------------Models---------------------------
#rosservice call gazebo/get_world_properties
MODELS = ['cube1','cube2','cube3','cube4','cube5']
#----------------Parameters------------------------
RATE = 	4
MIN_SPEED = 1
MAX_SPEED = 2
conveySpeed = 1
conveyor_start=[4,0.25,0.075]
#pick_pos=[0,0.25,0.0725]
#place_pos=

class conveyor():
	def __init__(self):
		#Initialize node
		rospy.init_node('conveyor_belt', anonymous=True)
		rate = rospy.Rate(RATE)

		# When the user quits, call the shutdown function:
		rospy.on_shutdown(self.shutdown)	

		#Wait for services
		rospy.wait_for_service('/gazebo/get_model_state')
		rospy.wait_for_service('/gazebo/set_model_state')
		print("Services Ready")

		#Define handlers to call each service
		self.call_get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
		self.call_set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

		while not rospy.is_shutdown():
			#Move one model at a time
			for model_name in MODELS:	
				#Start with stationary model, then move onto conveyor, then move on the conveyor belt
				(cmd, nextTime) = self.setStationary(model_name,1)
				isMoving = False	
				#Initialize in position to false, once it's true, stop the while loop
				inposition = False
				
				#Repeat until the model is in place on the conveyor belt
				while not inposition:
					pubCmd = False
					#After duration has passed		
					if (rospy.Time.now() >= nextTime):
						# It's now time to publish a command
						pubCmd = True
						# We want the model to move now.	
						(cmd, nextTime) = self.setMove(model_name)
						isMoving = True
					else:
						if not isMoving:
							# If we're not moving, 
							# we want to keep publishing the same command
							# to keep the model in place.
							pubCmd = True

					if (pubCmd):
						# Issue the service request to move the model:
						myResponse = self.call_set_model_state(cmd)
						# myResponse.success will be 'True' if everything is OK.
						# Otherwise, print an error message:
						if (not myResponse.success):
							print('Error:')
							print(myResponse.status_message)

						#Condition for in position is if z value of pose is > 0.04
						#NOTE Subject to change
						[posx,posy,posz] = self.whereAmI(model_name)
						if posz > 0.072 and posy <0.3:	
							inposition = True
							print(model_name," is in Position")

						rate.sleep()
				
				#We then move the model along the conveyor belt with the aim that the robotic arm picks it up 
				picked = False
				isMoving = False
				(cmd, nextTime) = self.setStationary(model_name,3)
				print('GOAL: Get picked by robotic arm')
				while not picked:
					# Assume that we're not going to publish a command
					pubCmd = False		
					if (rospy.Time.now() >= nextTime):
						# It's now time to publish a command
						pubCmd = True
						# We want the model to move now.	
						(cmd, nextTime) = self.convey(model_name)
						isMoving = True
					else:
						if not isMoving:
							# If we're not moving, 
							# we want to keep publishing the same command
							# to keep the model in place.
							pubCmd = True

					if (pubCmd):
						# Issue the service request to move the model:
						myResponse = self.call_set_model_state(cmd)
						# print myResponse
			
						# myResponse.success will be 'True' if everything is OK.
						# Otherwise, print an error message:
						if (not myResponse.success):
							print('Error:')
							print(myResponse.status_message)

						#We assume it was picked up by the arm if the x value of the pose is greater than 0.1
						#NOTE Subject to change
						[posx,posy,posz] = self.whereAmI(model_name)
						if posz > 0.1:	
							picked = True
							print(model_name," was picked by robotic arm")

						rate.sleep()			
	#Find the position of a model
	def whereAmI(self,model_name):
		# 1) Call Gazebo Service:
		modelLoc = self.call_get_model_state(model_name, '')
		# 2) Grab just the info we need:
		[x, y, z] = [modelLoc.pose.position.x, modelLoc.pose.position.y, modelLoc.pose.position.z]
		# 3) Return this info:
		return (x, y, z)
	
	#Keep model stationary for 1s
	def setStationary(self,model_name,duration):
		# Where is the model right now?
		(x, y, z) = self.whereAmI(model_name)
		print(model_name,'is stationary at:', round(x,2), round(y,2), round(z,2))
		
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

		# print cmd		
		return (cmd, nextTime)

	#Move model to beginning of the conveyor belt
	def setMove(self, model_name):
		# Where is the model right now?
		(xNow, yNow, zNow) = self.whereAmI(model_name)
		print(model_name," is at",round(xNow,2),round(yNow,2),round(zNow,2))

		# Where do you want it to go?
		(xGoal, yGoal, zGoal) = conveyor_start
		print('GOAL:', xGoal, yGoal, zGoal)
		
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
		#xgoal < xnow so we're moving in the -ve x direction
		cmd.twist.linear.x	= (xGoal - xNow) / duration
		#ygoal > ynow so we're moving in the +ve y direction
		cmd.twist.linear.y	= (yGoal - yNow) / duration
		#zgoal > znow so we're moving in the +ve z direction
		cmd.twist.linear.z	= 2 / duration

		return (cmd, nextTime)

	#Move model along conveyor belt
	def convey(self, model_name):
		# Where is the model right now?
		(xNow, yNow, zNow) = self.whereAmI(model_name)
		print("Model is at",round(xNow,2),round(yNow,2),round(zNow,2))

		# Where do you want it to go?
		xGoal = 0
		
		# How far is it from the current location to the goal location?
		dist = xNow - xGoal
				
		# How long will it take to get to the goal location?
		duration = dist/conveySpeed		

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
		
		return (cmd, nextTime)	

	#Shutdown Function
	def shutdown(self):
		rospy.loginfo("Shutting down the node...")
		rospy.sleep(1)


if __name__ == "__main__":
	try:
		conveyor()
	except rospy.ROSInterruptException:
		pass

#make conditions for picked and in position better (2 dimensional)

