#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import sympy as sym
from sympy import *
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#########################################################################################################

#########################################################################################################

#######################################################################
#Initialize ROS Node
rospy.init_node('Fuzzy_Rob_2D', anonymous=True) #Identify ROS Node
#####################################################################

#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Identify the publisher "pub1" to publish on topic "/cmd_vel" to send message of type "Twist"
vel_msg = Twist() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont_1 = 0	#Initialize flag by zero
pos_des_msg = Pose()	#Identify msg variable of data type Pose
position_des = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback_Des_Pos(data):
  global pos_des_msg	#Identify msg variable created as global variable
  global sub0		#Identify a subscriber as global variable
  global flag_cont_1
  global position_des 

  msg = data
  pos_msg.position.x = round(msg.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position_des = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_cont_1 = 1

sub0 = rospy.Subscriber('/APF_Des_Pos', Pose, callback_Des_Pos) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback_Act_Pos(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub1		#Identify a subscriber as global variable
  global flag_cont
  global position 

  msg = data
  pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_cont = 1

sub1 = rospy.Subscriber('/odom', Odometry, callback_Act_Pos) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Act_Pos_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub2			#Identify a subscriber as global variable
  global flag_initial 	#Identify flag created as global variable
  global position_0 

  msg = data
  pos_msg_0.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg_0.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_initial = 1
  sub2.unregister()				#Unsubsribe from this topic

sub2 = rospy.Subscriber('/odom', Odometry, callback_Act_Pos_Init) #Identify the subscriber "sub1" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
#######################################################################

#######################################################################
#Define the initial pose and velocity of the vehicle
Rob_pos_0 = [position_0[0],position_0[1],position_0[3]]
#######################################################################
#########################################################################################################


#########################################################################################################
## Fuzzy Implementation
def Fuzzy_Ctrl(Goal_pos,Act_pos):
	# New Antecedent/Consequent objects hold universe variables and membership functions
	Distance = ctrl.Antecedent(np.arange(0, 100+0.1, 0.1), 'Distance') #Input
	Angle = ctrl.Antecedent(np.arange(-np.pi, np.pi+0.001, 0.001), 'Angle') #Input

	Velocity = ctrl.Consequent(np.arange(-0.13, 0.26+0.13+0.01, 0.01), 'Velocity') #Output
	Omega = ctrl.Consequent(np.arange(-1.82-1.82-0.01, 1.82+1.82+0.01, 0.01), 'Omega') #Output

	# Auto-membership function population is possible with .automf(3, 5, or 7)
	#Distance.automf(5)
	#Velocity.automf(3)

	# Custom membership functions can be built interactively with a familiar, Pythonic API
	Distance['Near'] = fuzz.trimf(Distance.universe, [0, 0, 0.5])
	Distance['Mid'] = fuzz.trimf(Distance.universe, [0, 0.5, 1])
	Distance['Far'] = fuzz.trapmf(Distance.universe, [0.5, 1, 100, 100])

	Angle['Neg'] = fuzz.trapmf(Angle.universe, [-np.pi, -np.pi, -np.pi/4, 0])
	Angle['Zero'] = fuzz.trimf(Angle.universe, [-np.pi/4, 0, np.pi/4])
	Angle['Pos'] = fuzz.trapmf(Angle.universe, [0, np.pi/4, np.pi, np.pi])

	Velocity['Zero'] = fuzz.trimf(Velocity.universe, [-0.13, 0, 0.13])
	Velocity['Half'] = fuzz.trimf(Velocity.universe, [0, 0.13, 0.26])
	Velocity['Full'] = fuzz.trimf(Velocity.universe, [0.13, 0.26, 0.26+0.13])

	Omega['Negative'] = fuzz.trimf(Omega.universe, [-1.82-1.82, -1.82, 0])
	Omega['Zero'] = fuzz.trimf(Omega.universe, [-1.82, 0, 1.82])
	Omega['Positive'] = fuzz.trimf(Omega.universe, [0, 1.82, 1.82+1.82])

	# You can see how these look with .view()
	#Distance.view()
	#Angle.view()

	#Velocity.view()
	#Omega.view()

	#Identify Rules
	#rule0 = ctrl.Rule(antecedent=((Distance['nb'] & delta['nb']) |
	#                              (Distance['ns'] & delta['nb']) |
	#                              (Distance['nb'] & delta['ns'])),
	#                  consequent=output['nb'], label='rule nb')
	rule1 = ctrl.Rule(Angle['Neg'], Velocity['Zero'])
	rule2 = ctrl.Rule(Angle['Neg'], Omega['Negative'])
	rule3 = ctrl.Rule(Angle['Pos'], Velocity['Zero'])
	rule4 = ctrl.Rule(Angle['Pos'], Omega['Positive'])
	rule5 = ctrl.Rule(Angle['Zero'], Omega['Zero'])
	rule6 = ctrl.Rule(antecedent=((Distance['Near'] & Angle['Zero'])),
		          consequent=Velocity['Zero'])
	rule7 = ctrl.Rule(antecedent=((Distance['Mid'] & Angle['Zero'])),
		          consequent=Velocity['Half'])
	rule8 = ctrl.Rule(antecedent=((Distance['Far'] & Angle['Zero'])),
		          consequent=Velocity['Full'])

	Robot_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
	#Robot_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule6, rule7, rule8])
	Motion = ctrl.ControlSystemSimulation(Robot_ctrl)

	# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
	# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
	dis = np.sqrt((Goal_pos[0]-Act_pos[0])**2+(Goal_pos[1]-Act_pos[1])**2)
	print('distance is',dis)
	Motion.input['Distance'] = dis
	global last
	ang = np.arctan2((Goal_pos[1]-Act_pos[1]),(Goal_pos[0]-Act_pos[0]))-Act_pos[2]
	while ang < last-np.pi: ang += 2*np.pi
	while ang > last+np.pi: ang -= 2*np.pi
	last = ang
	print('angle is',ang)
	Motion.input['Angle'] = ang

	# Crunch the numbers
	Motion.compute()

	print(Motion.output['Velocity'])
	#Velocity.view(sim=Motion)
	out1 = Motion.output['Velocity']

	print(Motion.output['Omega'])
	#Omega.view(sim=Motion)
	out2 = Motion.output['Omega']

	Cont_input = [out1,out2]
	print(Cont_input)
	return Cont_input
#########################################################################################################

#########################################################################################################
#######################################################################
#Inputs
tau = 0.1 #Sampling Time
rob_mass = 1.8 #Robot Mass (Turtlebot 3 Waffle_pi)
last = 0
while 1 and not rospy.is_shutdown():
    if flag_cont == 1 and flag_cont_1 == 1:
	#Get Robot Current Position and Velocity
	Rob_pos = [position[0],position[1],position[3]]
	Rob_pos_des = [position_des[0],position_des[1],position_des[3]]

	#Calculate the control effort from the Lyaponov-based position control law	
	Cont_input = Fuzzy_Ctrl(Rob_pos_des,Rob_pos)

        flag_cont = 0
	flag_cont_1 = 0
    else:
        #Set the values of the Twist msg to be publeshed
        Cont_input = [0,0]

    v = round(float(Cont_input[0]),2) 	#Linear Velocity	
    w = round(float(Cont_input[1]),2)	#Angular Velocity
    #Set the values of the Twist msg to be publeshed
    vel_msg.linear.x = v #Linear Velocity
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = w #Angular Velocity
    pub1.publish(vel_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
#########################################################################################################
