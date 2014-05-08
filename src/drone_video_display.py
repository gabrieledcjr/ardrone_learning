#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_learning')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
'''from PySide import QtCore, QtGui'''

# We need an image convereted to use opencv
# GABE
import cv2.cv as cv
import cv2 as cv2
from image_converter import ToOpenCV, ToRos
import numpy as np
from std_msgs.msg import Int32, Float32, Bool

# Some Constants
'''CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms'''
CONNECTION_CHECK_PERIOD = 2.250
GUI_UPDATE_PERIOD = 0.20
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


'''class DroneVideoDisplay(QtGui.QMainWindow):'''
class DroneVideoDisplay():
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		'''super(DroneVideoDisplay, self).__init__()'''

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		'''self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)'''

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()
		# GABE
                cv.NamedWindow("windowimage", cv.CV_WINDOW_AUTOSIZE)

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		'''self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)'''
                # GABE
                self.connectionTimer = rospy.Timer(rospy.Duration(CONNECTION_CHECK_PERIOD), self.ConnectionCallback)
		
		# A timer to redraw the GUI
		'''self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)'''
                self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.RedrawCallback)

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self, event):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self, event):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					'''image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))'''
                                        image_cv = ToOpenCV(self.image)
					'''if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()'''
			finally:
				self.imageLock.release()

                        # GABE
                        # Convert BGR to HSV
                        frame = np.asarray(image_cv)
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# ADJUST these values to get a greater range of color
                        lower_gray = np.array([ 8, 6, 71], dtype=np.uint8)
                        upper_gray = np.array([50,45,135], dtype=np.uint8)

			lower_white = np.array([ 0, 0,245], dtype=np.uint8)
                        upper_white = np.array([10,10,255], dtype=np.uint8)
                        # Threshold the HSV image to get only white color
                        goal_mask = cv2.inRange(hsv, lower_white, upper_white)

                        # Threshold the HSV image to get only gray colors
                        mask = cv2.inRange(hsv, lower_gray, upper_gray)
                        flag, mask = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
                        flag, goal_mask = cv2.threshold(goal_mask, 100, 255, cv2.THRESH_BINARY)                        
 
                        # NEEDS TO PUBLISH Percentage, size, number of gray
                        number = cv2.countNonZero(mask)
                        size = mask.shape[0] * mask.shape[1]
                        percentage = (number * 1.0) / (size * 1.0)
			pubSize = rospy.Publisher('/ardrone_learning/SizeGray', Int32)
                        pubPercent = rospy.Publisher('/ardrone_learning/PercentGray', Float32)        
                        pubSize.publish(data=number)
			pubPercent.publish(data=percentage)

			goal_number = cv2.countNonZero(goal_mask)
                        goal_percentage = (goal_number * 1.0) / (size * 1.0)

			rospy.loginfo("Gray Percentage: %s", str(percentage))
                        pubGoal = rospy.Publisher('/ardrone_learning/IsGoal', Bool)
			if goal_percentage >= 45.0:
				pubGoal.publish(data=True)
			else:
				pubGoal.publish(data=False)                       
 
                        cv2.imshow("goal_mask", goal_mask)
                        cv2.imshow("mask", mask)
                        cv.ShowImage("windowimage", image_cv)
                        cv.WaitKey(2)
			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			'''self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)'''

		# Update the status bar to show the current drone status & battery level
		'''self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)'''

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		'''msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))'''

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	'''app = QtGui.QApplication(sys.argv)'''
	display = DroneVideoDisplay()
	'''display.show()'''
	'''status = app.exec_()'''
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
