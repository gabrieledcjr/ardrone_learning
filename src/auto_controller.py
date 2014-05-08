#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_learning')
import rospy
import os
import errno
import argparse
import random

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
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
from std_msgs.msg import Float32 
from std_msgs.msg import Bool

class State:
   def __init__(self):
      self.x = 0
      self.y = 0
      self.actions = {'forward': 0, 'backward': 0, 'left': 0, 'right': 0,
          'turn_right': 0, 'turn_left': 0}
      self.special = None

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
      self.pitch += 1.5
      self.UpdateDroneState(pitch=self.pitch)
      x -= 1
    #self.pitch -= 1.0
    #self.UpdateDroneState(pitch=self.pitch)

  # FIXME convert x to meters
  def Backward(self, x):
    while x > 0:
      self.pitch += -1.5
      self.UpdateDroneState(pitch=self.pitch)
      x -= 1
    #self.pitch -= -1.0
    #self.UpdateDroneState(pitch=self.pitch)

  # FIXME convert x to meters
  def Left(self, x):
    while x > 0:
      self.roll += 1.5
      self.UpdateDroneState(roll=self.roll)
      x -= 1
    #self.roll -= 1.0
    #self.UpdateDroneState(roll=self.roll)

  # FIXME convert x to meters
  def Right(self, x):
    while x > 0:
      self.roll += -1.5
      self.UpdateDroneState(roll=self.roll)
      x -= 1
    #self.roll -= -1.0
    #self.UpdateDroneState(roll=self.roll)

  # FIXME convert x to degrees
  def TurnLeft(self, x):
    while x > 0:
      self.yaw_velocity += 1.5
      self.UpdateDroneState(yaw=self.yaw_velocity)
      x -= 1
    #self.yaw_velocity -= 1.0
    #self.UpdateDroneState(yaw=self.yaw_velocity)

  # FIXME convert x to degrees
  def TurnRight(self, x):
    while x > 0:
      self.yaw_velocity += -1.5
      self.UpdateDroneState(yaw=self.yaw_velocity)
      x -= 1
    #self.yaw_velocity -= -1.0
    #self.UpdateDroneState(yaw=self.yaw_velocity)

  # FIXME convert x to meters
  def Up(self, x):
    while x > 0:
      self.z_velocity += 1.5
      self.UpdateDroneState(altitude=self.z_velocity)
      x -= 1
    #self.z_velocity -= 1.0
    #self.UpdateDroneState(altitude=self.z_velocity)

  # FIXME convert x to meters
  def Down(self, x):
    while x > 0:
      self.z_velocity += -1.5
      self.UpdateDroneState(altitude=self.z_velocity)
      x -= 1
    #self.z_velocity -= -1.0
    #self.UpdateDroneState(altitude=self.z_velocity)

  def UpdateDroneState(self, roll=0, pitch=0, yaw=0, altitude=0):
    self.roll = roll
    self.pitch = pitch
    self.yaw_velocity = yaw
    self.z_velocity = altitude
    self.controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, 
        self.z_velocity)
    time.sleep(1)


class Experiment():

  def __init__(self):
    self.display = DroneVideoDisplay()
    self.controller = ARDroneController()

    reset_model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState)

    self.subPoseData = rospy.Subscriber('/gazebo/model_states', ModelStates, 
        self.ReceiveModelStates)
    #rospy.Subscriber('targetSeen', String, self.callback1)
    #rospy.Subscriber('targetX', String, self.callback2)
    #rospy.Subscriber('targetY', String, self.callback3)
    rospy.Subscriber('/ardrone_learning/PercentGray', Float32, self.PercentGray)
    rospy.Subscriber('/ardrone_learning/IsGoal', Bool, self.IsGoal)

    width = 8
    height = 20

    self.__width = width
    self.__height = height
    self.__current_state = (0,0)
    self.__default_q = 1
    self.q_map = q_map
    #self.__actions = {'forward', 'backward', 'left', 'right', 'turn_right', 
    #    'turn_left'}
    self.__actions = {'forward', 'left', 'right', 'turn_right', 'turn_left'}
    self.world = self.__build_world(width, height)
    self.__total_reward = 0
    self.__episode_reward = 0
    self.__turn = 0
    self.end_episode = False


  def ReceiveModelStates(self, model_states):
    self.x_position = int(model_states.pose[7].position.x)
    self.y_position = int(model_states.pose[7].position.y)

    # Determine state
    self.__current_state = (self.x_position, self.y_position)


  def PercentGray(self, gray):
    #rospy.loginfo('gray: {}'.format(gray))
    if gray.data >= 0.06:
      # Crashing into a wall end episode
      reward = -100
      self.__total_reward -= 100
      self.__episode_reward -= 100
      self.end_episode = True


  def __build_world(self, width, height):
    grid = []
    # Build the rest of the rows
    for _ in range(height):
      row = []
      # Build a single row
      for _ in range(width):
        row.append(State())
      grid.append(row)

    return grid


  def __get_q(self, x, y, action):
    return self.world[x][y].actions[action]


  def __set_q(self, x, y, action, reward):
    self.world[x][y].actions[action] = reward


  def __get_max_action(self, x, y):

    for value in self.world[x][y].actions.values():
      if value != self.__default_q:
        action = max(self.world[x][y].actions, key=self.world[x][y].actions.get)
        return action[0]

  """
  <<<<<<< Updated upstream
    def ReceiveModelStates(self, model_states):
      self.x_position = model_states.pose[10].position.x
      self.y_position = model_states.pose[10].position.y
      #rospy.loginfo("The values of x & y : %s %s",str(self.x_position), str(self.y_position))
  =======
      return 'N'
  >>>>>>> Stashed changes
  """
  def __get_max_action_value(self, x, y):

    for value in self.world[x][y].actions.values():
      if value != self.__default_q:
        action = max(self.world[x][y].actions, key=self.world[x][y].actions.get)
        return self.world[x][y].actions[action]

    return 'N'

  def __choose_action(self, state):
    # Take the highest Q action
    # Default to a huge negative number to force it to be reset
    x, y = state
    greedy_q_value = -10000000000
    for action in self.world[x][y].actions:
      new_q_value = self.__get_q(x, y, action)
      if  new_q_value > greedy_q_value:
        greedy_action = action
        greedy_q_value = new_q_value

    # 10% of the time take an exploratory action instead
    percent = random.randint(1, 10)

    # Explore action
    if percent == 1:
      action = self.__explore(greedy_action)
    else:
      action = greedy_action

    return action


  def __explore(self, exclude_action):
    explore_actions = []
    for action in self.__actions:
      if action != exclude_action:
        explore_actions.append(action)

    return random.choice(explore_actions)


  def __move(self, turn, action, f):
    # Set the new state as the current in case none of the actions are valid
    new_state = self.__current_state
    old_state = self.__current_state
    x, y = self.__current_state

    if action == 'forward':
      self.controller.Forward(1)

    #if action == 'backward':
    #  self.controller.Backward(1)

    if action == 'left':
      self.controller.Left(1)

    if action == 'right':
      self.controller.Right(1)

    if action == 'turn_left':
      self.controller.TurnLeft(1)

    if action == 'turn_right':
      self.controller.TurnRight(1)

  def IsGoal(self, is_goal):
    # If the new state is the goal, hooray! 100 points

    if is_goal.data is True:
      # Goal state found!
      reward = 100
      self.__total_reward += 100
      self.__episode_reward += 100
      self.record_move(self.__turn, x, y, action, reward, f)

      # Record end game values
      self.record_end_game(f, True)

  def get_reward(self, action, f):
    # Haven't found the goal yet keep looking -1 point
    x, y = self.__current_state
    reward = -1
    self.__total_reward -= 1
    self.__episode_reward -= 1
    self.record_move(self.__turn, x, y, action, reward, f)

    return reward

  def episode(self, max_num_turns, q_map, f, alpha, gamma):
    if q_map is not None:
      self.world = q_map

    # Choose action from state using policy derived from Q (e.g. e-greedy)
    action = self.__choose_action(self.__current_state)

    # Repeat (for each step of episode):
    self.__turn = -1

    #for turn in range(max_num_turns):
    while max_num_turns:

      self.__turn += 1
      # Take action a, observe r, s'
      old_state = self.__current_state
      old_action = action
      self.__move(self.__turn, action, f)
      # Wait for move command to go into effect
      #time.sleep(1)

      if self.end_episode is True:
        reward = -100

        new_state = self.__current_state
        # Choose action' from state' using policy derived from Q(e.g. e-greedy)
        new_action = self.__choose_action(new_state)

        qsa = self.__get_q(old_state[0], old_state[1], old_action)
        qsa_new = self.__get_q(new_state[0], new_state[1], new_action)
        updated_reward = qsa + (alpha * (reward + gamma * qsa_new - qsa))
        self.__set_q(old_state[0], old_state[1], old_action, updated_reward)
        return self.world

      reward = self.get_reward(action, f)

      new_state = self.__current_state
      # Choose action' from state' using policy derived from Q(e.g. e-greedy)
      new_action = self.__choose_action(new_state)

      # Q(s,a) <- Q(s,a) + alpha[r + gamma * Q(s',a') - Q(s,a)]
      qsa = self.__get_q(old_state[0], old_state[1], old_action)
      qsa_new = self.__get_q(new_state[0], new_state[1], new_action)
      updated_reward = qsa + (alpha * (reward + gamma * qsa_new - qsa))
      self.__set_q(old_state[0], old_state[1], old_action, updated_reward)
      # s <- s'; a <- a';
      action = new_action
      rospy.loginfo("#: {} Action: {}".format(max_num_turns, action))
      max_num_turns -= 1
      time.sleep(1)

    # If we get here, we maxed out the number of turns
    print("Max turns reached before goal state :(")
    # Record end game values
    self.record_end_game(f, False)


  def record_move(self, turn, x, y, action, reward, f):
    move = ', '.join([str(turn), str(x), str(y), action, str(reward)])
    f.write(move + '\n')


  def record_end_game(self, f, goal):
    f.write("Total Reward: " + str(self.__total_reward) + '\n')
    f.write("Episode Reward: " + str(self.__episode_reward) + '\n')
    # Reset the episode reward 
    self.__episode_reward = 0

    # Print value map of best actions
    for y in range(self.__height):
      # Solid line across
      for _ in range(41):
        f.write('-')
      f.write('\n')
      # Vertical lines and values
      f.write('| ')
      for x in range(self.__width):
        f.write(self.__get_max_action(x,y))
        f.write(' | ')
      f.write('\n') 

    # Solid line across
    for _ in range(41):
      f.write('-')
    f.write('\n')
    f.write('\n')

    self.record_end_game_values (f)


  def record_end_game_values(self, f):
    # Print value map of best actions
    for y in range(self.__height):
      # Solid line across
      for _ in range(41):
        f.write('-')
      f.write('\n')
      # Vertical lines and values
      f.write('| ')
      for x in range(self.__width):
        action_value = self.__get_max_action_value(x,y)
        if action_value == 'N':
          action_value = 100
        f.write('{:.2f}'.format(action_value))
        f.write(' | ')
      f.write('\n') 

    # Solid line across
    for _ in range(41):
      f.write('-')
    f.write('\n')


  #def episode(self):
  #  time.sleep(1)
  #  self.controller.Takeoff()
  #  self.controller.Forward(1)


# Setup the application
if __name__=='__main__':
  import sys
  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('ardrone_auto_controller', anonymous=True)

  # Now we construct our Qt Application and associated controllers and windows
  '''app = QtGui.QApplication(sys.argv)'''

  """
  inputs = argparse.ArgumentParser(description = "Ardrone Learning")
  inputs.add_argument('trial', metavar = 'trial', type = int, 
      help = "The trial run number")
  inputs.add_argument('-alpha', '--alpha', metavar='alpha', type=float, 
      help="The alpha used in learning")
  inputs.add_argument('-gamma', '--gamma', metavar='gamma', type=float, 
      help="The gamma used in learning")
  args = inputs.parse_args()
  if args.alpha:
    alpha = args.alpha
  else:
    alpha = 0.1
  if args.gamma:
    gamma = args.gamma
  else:
    gamma = 0.5

  """
  episodes = 250000
  max_num_turns = 10000000000
  q_map = None
  alpha = 0.1
  gamma = 0.5

  controller = ARDroneController()
  #experiment = Experiment()

  path = str(3)
  try:
    os.makedirs(path)
  except OSError as exc: 
    if exc.errno == errno.EEXIST and os.path.isdir(path):
      pass
    else: 
      raise exc

  for episode in range(episodes):
    experiment = Experiment()
    f = open('{}/{}_gridworld_results_{}.txt'.format(3, 3,
        episode), 'w')
    controller.Takeoff()
    time.sleep(3)
    q_map = experiment.episode(max_num_turns, q_map, f, alpha, gamma)
    # Reset the quadcopter
    controller.Land()
    time.sleep(4)
    os.system("rosservice call /gazebo/set_model_state '{model_state: "
        "{ model_name: quadrotor, pose: { position: { x: 0, y: 0, z: 0.1 }, "
        "orientation: {x: 0, y: 0, z: 0, w: 1} }, twist: "
        "{linear: {x: 0, y: 0, z: 0 }, angular: {x: 0, y: 0, z: 0} }, "
        "reference_frame: world } }'")
    del experiment
    f.close()
    time.sleep(1)

	# and only progresses to here once the application has been shutdown
	#rospy.signal_shutdown('Great Flying!')
	#sys.exit()
  rospy.spin()
  #while 1:
  #time.sleep(5)
