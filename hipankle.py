import motion
import argparse
import time
import almath
import math
from threading import Thread
import matplotlib.pyplot as plt
import pickle

from naoqi import ALProxy

xList = []
accList = []
polyList = []
accList1 = []
polyList1 = []
forceList = []
yList = []

#Getting angles of the leg from the sensors
def CoMBalancing():

	robotIP = "129.125.178.114"
	PORT = 9559
	memoryProxy = ALProxy("ALMemory", robotIP, PORT)
	motionProxy = ALProxy("ALMotion", robotIP, PORT)
	postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

	while(True):
		
		print 'Tasklist: ', motionProxy.getTaskList()
		name ="LLeg"
		space = motion.FRAME_ROBOT
		useSensorValues = True
		result = motionProxy.getPosition(name, space, useSensorValues)
		LeftLeg = result
		name = "RLeg"
		result = motionProxy.getPosition(name, space, useSensorValues)
		RightLeg = result

		if LeftLeg[0] > RightLeg[0]:
			LeftLegForward = True
			legName = "LLeg"
		else:
			LeftLegForward = False
			legName = "RLeg"

		#Lines of support polygon - (x1,y1)(x2,y2) and	(x3,y3)(x4,y4) 
		widthOfLeg = 7.5 / 200.0
		lengthOfLeg = 15.0 / 200.0

		if (LeftLegForward):
			y1 = LeftLeg[1] - widthOfLeg 
			x1 = LeftLeg[0] + lengthOfLeg 
			y3 = LeftLeg[1] + widthOfLeg 
			x3 = LeftLeg[0] - lengthOfLeg
			y2 = RightLeg[1] - widthOfLeg
			x2 = RightLeg[0] + lengthOfLeg
			y4 = RightLeg[1] + widthOfLeg
			x4 = RightLeg[0] - lengthOfLeg
		else:
			y1 = LeftLeg[1] + widthOfLeg
			x1 = LeftLeg[0] + lengthOfLeg
			y3 = LeftLeg[1] - widthOfLeg
			x3 = LeftLeg[0] - lengthOfLeg
			y2 = RightLeg[1] + widthOfLeg
			x2 = RightLeg[0] + lengthOfLeg
			y4 = RightLeg[1] - widthOfLeg
			x4 = RightLeg[0] - lengthOfLeg

		if (x2 != x1):
			m1 = (x2 - x1) / (y2 - y1)
			b1 = x2 - m1 * y2

		if (x3 != x4):
			m2 = (x4 - x3) / (y4 - y3)
			b2 = x4 - m2 * y4


		#comparing the position of CoM with area of support polygon
		name = "Body"
		useSensors = True
		pos = motionProxy.getCOM(name, motion.FRAME_ROBOT, useSensors)

		xCoM = pos[0]
		yCoM = pos[1]
		
		AccelX = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
		
		xList.append(xCoM)
		accList.append(b1*0.6)
		polyList.append(b2*0.2)
		accList1.append(b1)
		polyList1.append(b2)
		forceList.append(AccelX)  ### Normalize data to fit plot
		yList.append(time.time())

		if (xCoM > (yCoM * m1 + b1) * 0.6):
		#if ((xCoM + 0.0125*AccelX > (yCoM * m1 + b1))):  # Good values: 0.6 and 1.8 (For leaning only)
			print "AccelX: ", AccelX


			#position(-0.01, -0.1)
			motionProxy.killMove()
			sit(LeftLegForward, xCoM)    ### Comment this out for experiments
			

			print "Balancer activated"
			
			print "x = %.2f y + %.2f" % (m1, b1)
			print "x = %.2f y + %.2f" % (m2, b2)

			print ("XCoM %.2f  yCoM %.2f" ) % (xCoM, yCoM)
			#postureProxy.goToPosture("StandInit", 0.5)
			time.sleep(5.0)

		if (xCoM < (yCoM * m2 + b2) * 0.2):
			pass



def sit(leftFootFoward, xCom):

	print "Sitting: ", xCom
	motionProxy = ALProxy("ALMotion", "129.125.178.114", 9559)

	effector = "Torso"
	frame = motion.FRAME_ROBOT
	useSensorValues = True
	torso = motionProxy.getPosition(effector, frame, useSensorValues)
	print "Torso:", torso

	LAngles = motionProxy.getAngles(["LHipPitch", "LKneePitch", "LAnklePitch"], True)
	RAngles = motionProxy.getAngles(["RHipPitch", "RKneePitch", "RAnklePitch"], True)

	### Adjust parameter control
	parameter = 0.24 ###xCom*4

	AnkleAngle = 0.3
	### De facto value: -0.08 for changeAngles
	if(leftFootFoward):
		LHip = -1*LAngles[1]-(LAngles[2]+AnkleAngle+0.25)/2
		RHip = -1*RAngles[1]-(LAngles[2]+AnkleAngle+0.35)/2
	else:
		LHip = -1*LAngles[1]-(LAngles[2]+AnkleAngle+0.35)/2
		RHip = -1*RAngles[1]-(LAngles[2]+AnkleAngle+0.25)/2

	HipAnkle = ["LAnklePitch", "RAnklePitch", "LHipPitch", "RHipPitch"]
	#JointAngles = [0.1+parameter, 0.1+parameter, LHip+parameter, RHip+parameter]
	JointAngles = [AnkleAngle, AnkleAngle, LHip+parameter, RHip+parameter]
	TimeList = [0.2, 0.2, 0.3, 0.3]

	motionProxy.angleInterpolation(HipAnkle, JointAngles, TimeList, False)
		
	
def step(robotIP):

	motionProxy = ALProxy("ALMotion", robotIP, 9559)
	legName = ["LLeg"]
	X = 0.04 # max 0.08 min -0.04
	Y = 0.00	# max 0.16
	Theta = 0.0
	footSteps = [[X, Y, Theta]]
	timeList = [0.1]
	clearExisting = False

	#time.sleep(0.2)
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
	motionProxy.waitUntilMoveIsFinished()


	time.sleep(0.2)
	legName = ["RLeg"]
	X = 0.0
	footSteps = [[X, Y, Theta]]
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
	motionProxy.waitUntilMoveIsFinished()
	time.sleep(0.2)
	legName = ["RLeg"]
	X = 0.04
	footSteps = [[X, Y, Theta]]
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
	motionProxy.waitUntilMoveIsFinished()
	time.sleep(0.2)
	legName = ["LLeg"]
	X = 0.0
	footSteps = [[X, Y, Theta]]
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
	motionProxy.waitUntilMoveIsFinished()

def initArmPose(robotIP, PORT=9559):
	motionProxy = ALProxy("ALMotion", robotIP, PORT)

	names = ["LElbowRoll", "RElbowRoll", "LShoulderPitch", "RShoulderPitch"]

	angles = [-1.4, 1.4, 1.0, 1.0]
	times = [0.2, 0.2, 0.2, 0.2]
	motionProxy.setAngles(names, angles, 0.1)

	motionProxy.setMoveArmsEnabled(False, False)

def main(robotIP, PORT=9559):

	motionProxy = ALProxy("ALMotion", robotIP, PORT)
	postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

	motionProxy.wakeUp()
	postureProxy.goToPosture("StandInit", 0.5)
	audioProxy = ALProxy("ALAudioDevice", robotIP, PORT)
	audioProxy.setOutputVolume(45)

	time.sleep(2)
	motionProxy.setFallManagerEnabled(False)
	

	t = Thread(target=CoMBalancing)

	t.setDaemon(True)
	t.start()

	initArmPose("129.125.178.114", 9559)	### Could disturb balance

	#sit(True, 0.02)
	#time.sleep(10)
	runtime = time.time()
	useSensorValues = True
	result = motionProxy.getRobotPosition(useSensorValues)
	print "Robot Position", result


	# Example showing how to use this information to know the robot's diplacement.
	initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
	endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
	# Compute robot's' displacement
	robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
	vector = robotMove.toVector()
	### Use while loop and check for robotPosition
	while abs(vector[0]) < 1.5:
		step(robotIP)
		endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(useSensorValues))
		# Compute robot's' displacement
		robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
		vector = robotMove.toVector()

	#for i in range(6):
	#	step(robotIP)

	runtime = time.time() - runtime
	data = {'xCom': xList, 'b1_custom': accList, 'b2_custom': polyList, 'b1': accList1, 'b2': polyList1, 'acc': forceList, 'time': yList, 'runtime': runtime, 'distance': vector[0]}

	### trial_default_X  (With box, no disturbance)
	### 10 trials for default
	### trial_disturbed_x (With box, disturbed)
	### 0 trial for disturbed
	with open('trial_default_11.pkl', 'wb') as f:
	 	pickle.dump(data, f)

	print "Time taken: ", runtime, " (s)"
	motionProxy.rest()

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="129.125.178.114", help="Robot ip address")
	parser.add_argument("--port", type=int, default=9559, help="Robot port number")
	args = parser.parse_args()
	main(args.ip, args.port)