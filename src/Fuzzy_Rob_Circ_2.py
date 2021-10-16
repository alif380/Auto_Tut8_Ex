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
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
Velocity_msg = Twist()
velocity = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global flag_cont
  global position 
  global Velocity_msg
  global velocity

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
  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]
  flag_cont = 1

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
Velocity_msg_0 = Twist()
velocity_0 = np.zeros((1,6))
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub1			#Identify a subscriber as global variable
  global flag_initial 	#Identify flag created as global variable
  global position_0 
  global Velocity_msg_0
  global velocity_0

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
  Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]
  flag_initial = 1
  sub1.unregister()				#Unsubsribe from this topic

sub1 = rospy.Subscriber('/odom', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
#######################################################################

#######################################################################
#Define the initial pose and velocity of the vehicle
Rob_pos_0 = [position_0[0],position_0[1],position_0[3]]
Roc_vel_0 = [velocity_0[0],velocity_0[5]]

x_p = Rob_pos_0[0]
y_p = Rob_pos_0[1]
vel_p_x = Roc_vel_0[0]*cos(Rob_pos_0[2])
vel_p_y = Roc_vel_0[0]*sin(Rob_pos_0[2])
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
r = rospy.get_param("~r_des");
w = rospy.get_param("~w_des");
time_sim = symbols('time_sim')
xs = [r*sym.sin(w*time_sim),r*sym.cos(w*time_sim)]
Goal_Pos = [0,0]
tau = 0.1 #Sampling Time
rob_mass = 1.8 #Robot Mass (Turtlebot 3 Waffle_pi)
last = 0
i = 0
sim_tim = 100
while 1 and not rospy.is_shutdown():
    if flag_cont == 1:
	#Get Robot Current Position and Velocity
	Goal_Pos[0] = float(xs[0].subs(time_sim,i*tau))
	Goal_Pos[1] = float(xs[1].subs(time_sim,i*tau))
	Rob_pos = [position[0],position[1],position[3]]
	Rob_vel = [velocity[0],velocity[5]]
	
	#Calculate the control effort from the fuzzy control
	Cont_input = Fuzzy_Ctrl(Goal_Pos,Rob_pos)
	v = round((Cont_input[0]),2) 	#Linear Velocity	
	w = round((Cont_input[1]),2)	#Angular Velocity

        #Set the values of the Twist msg to be publeshed
        vel_msg.linear.x = v #Linear Velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = w #Angular Velocity

	#Update the previous robot states for the next iteration
	vel_p_x = Rob_vel[0]*cos(Rob_pos[2])
	vel_p_y = Rob_vel[0]*sin(Rob_pos[2])
	x_p = Rob_pos[0]
	y_p = Rob_pos[1]
        flag_cont = 0
	i +=1
    else:
        #Set the values of the Twist msg to be publeshed
        vel_msg.linear.x = 0 #Linear Velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0 #Angular Velocity
    print('velocity is',v)
    pub1.publish(vel_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
#########################################################################################################
