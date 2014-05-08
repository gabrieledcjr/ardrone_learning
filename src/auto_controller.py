#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_learning')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
'''from PySide import QtCore, QtGui'''

# GABE
import cv2.cv as cv
import time

import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class ARDroneController():
  def __init__(self):
    self.pitch = 0.0
    self.roll = 0.0
    self.yaw_velocity = 0.0
    self.z_velocity = 0.0
    self.controller = BasicDroneController()
    self.UpdateDroneState()

  def Takeoff(self):
    self.controller.SendTakeoff()
    time.sleep(1)

  def Land(self):
    self.controller.SendLand()
    time.sleep(1)

  def Emergency(self):
    self.controller.SendEmergency()
    time.sleep(1)

  # FIXME convert x to meters
  def Forward(self, x):
    while x > 0:
      self.pitch += 1.0
      self.UpdateDroneState(pitch=self.pitch)
      x -= 1
    self.pitch -= 1.0
    self.UpdateDroneState(pitch=self.pitch)

  # FIXME convert x to meters
  def Backward(self, x):
    while x > 0:
      self.pitch += -1.0
      self.UpdateDroneState(pitch=self.pitch)
      x -= 1
    self.pitch -= -1.0
    self.UpdateDroneState(pitch=self.pitch)

  # FIXME convert x to meters
  def Left(self, x):
    while x > 0:
      self.roll += 1.0
      self.UpdateDroneState(roll=self.roll)
      x -= 1
    self.roll -= 1.0
    self.UpdateDroneState(roll=self.roll)

  # FIXME convert x to meters
  def Right(self, x):
    while x > 0:
      self.roll += -1.0
      self.UpdateDroneState(roll=self.roll)
      x -= 1
    self.roll -= -1.0
    self.UpdateDroneState(roll=self.roll)

  # FIXME convert x to degrees
  def TurnLeft(self, x):
    while x > 0:
      self.yaw_velocity += 1.0
      self.UpdateDroneState(yaw=self.yaw_velocity)
      x -= 1
    self.yaw_velocity -= 1.0
    self.UpdateDroneState(yaw=self.yaw_velocity)

  # FIXME convert x to degrees
  def TurnRight(self, x):
    while x > 0:
      self.yaw_velocity += -1.0
      self.UpdateDroneState(yaw=self.yaw_velocity)
      x -= 1
    self.yaw_velocity -= -1.0
    self.UpdateDroneState(yaw=self.yaw_velocity)
  
  # FIXME convert x to meters
  def Up(self, x):
    while x > 0:
      self.z_velocity += 1.0
      self.UpdateDroneState(altitude=self.z_velocity)
      x -= 1
    self.z_velocity -= 1.0
    self.UpdateDroneState(altitude=self.z_velocity)

  # FIXME convert x to meters
  def Down(self, x):
    while x > 0:
      self.z_velocity += -1.0
      self.UpdateDroneState(altitude=self.z_velocity)
      x -= 1
    self.z_velocity -= -1.0
    self.UpdateDroneState(altitude=self.z_velocity)

  def UpdateDroneState(self, roll=0, pitch=0, yaw=0, altitude=0):
    self.roll = roll
    self.pitch = pitch
    self.yaw_velocity = yaw
    self.z_velocity = altitude
    self.controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
    time.sleep(1)


'''
# episode(create("policy"), false, data, false)
class FeatureSet():
  self.FEATURES = 7 # How many
  self.d_values = [0.0] * self.FEATURES

  def __init__(self):
    pass
  #end __init__()

  def size(self):
    return len(d_values)
  #end size()
#end class FeatureSet


class QFunction():
  self.bias = 0.0  # constant feature
  self.ebias = 0.0 # constant eligbility feature 

  def __init__(self, prototype, filename=""):
    self.dl_weights = [0.0] * prototype.size()
    self.dl_eligibilty = [0.0] * prototype.size()
    
    if filename != "":
      fp = open(filename, "r")
      self.bias = float(fp.readline())
      for i in range(0, len(self.dl_weights)):
        self.dl_weights[i] = float(fp.readline())
      fp.close()
  #end __init__()
#end class QFunction


class QDrone():
  def __init__(self, proto):
    self.prototype = proto
    self.qFunction = QFunction(self.prototype)
  #end __init__()

  def loadPolicy(self, filename):
    self.qFunction = QFunction(self.prototype, filename)
  #end loadPolicy()
#end class QDrone()


class DataContainer():
  def __init__(self, i_step):
    self.step = i_step
  #end __init__()

  def getStep(self):
    return self.step
  #end getStep()

  def setStep(self, i_step):
    self.step = i_step
  #end setStep()
#end class DataContainer()


class Experiment():
  self.REPEATS = 10  # curves of average
  self.LENGTH = 300
  self.TEST = 30
  self.TRAIN = 10
  self.STEP = 500000
  self.data = DataContainer(self.STEP)

  def __init__(self):
    train("policy", 0)
  #end __init__()

  def create(self, s_learner):
    #@return RLDrone
    featureSet = FeatureSet()
    rlDrone = QDrone(featureSet)
    rlDrone.loadPolicy("data/" + s_learner)
    return rlDrone
  #end create()
    
  def train(self, s_learner, i_start):
    
    for i in range(start, self.REPEATS):
      rlDrone = self.create(s_learner)

      # Rest of the points
      for x in range(1, self.LENGTH):
        for y in range(0, self.TRAIN):
          self.episode(rlDrone, False)
        #d_score = self.evaluate(drone, TEST)
      
      # Save policy
      rlDrone.savePolicy("data/policy" + i)
  #end train() 

  def episode(self, rlDrone, testModel):
    i_steps = self.data.getStep()
    
    game = Game()
    rlDrone.startEpisode(game)
    while not game.isGoal():
      rlDrone.processStep(game)
      action = rlDrone.getAction(game)
      rlDrone.setAction(action)
      game.advanceGame(action)
      steps--
      if steps == 0:
        break
      time.sleep(1)

    self.data.setStep(steps)
    rlDrone.terminate()
    self.save() 
  #end episode() 
#end class Experiment
'''

class Experiment():
  
  def __init__(self):
    self.display = DroneVideoDisplay()
    self.controller = ARDroneController()
 
    self.subPoseData = rospy.Subscriber('/gazebo/model_states', ModelStates, self.ReceiveModelStates)

  def ReceiveModelStates(self, model_states):
    self.x_position = model_states.pose[10].position.x
    self.y_position = model_states.pose[10].position.y
    #rospy.loginfo("The values of x & y : %s %s",str(self.x_position), str(self.y_position))

  def episode(self):
    time.sleep(1)
    self.controller.Takeoff()
    self.controller.Forward(1)
    

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_auto_controller', anonymous=True)

	# Now we construct our Qt Application and associated controllers and windows
	'''app = QtGui.QApplication(sys.argv)'''
	#controller = BasicDroneController()
        
        #display = DroneVideoDisplay()
	#controller = ARDroneController()

        #time.sleep(5)
	#controller.Takeoff()
	#controller.Forward(1)
        #controller.TurnRight(30)

	experiment = Experiment()
        experiment.episode()

	# and only progresses to here once the application has been shutdown
	#rospy.signal_shutdown('Great Flying!')
	#sys.exit()
        rospy.spin()
        #while 1:
          #time.sleep(5)
