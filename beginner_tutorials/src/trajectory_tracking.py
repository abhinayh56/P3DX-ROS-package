#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import time

#temp
global Ith
Ith = 0

# time variables
global t_start
global t
t_start = 0
t = t_start

# current states
x   = 0
y   = 0
th  = 0
vx = 0
vy = 0
vth = 0

# vehicle current velocity
vc_now = 0
wc_now = 0

def getx0y0(t):
	x0i = 5.544444561
	y0i = 5.544444561
	
	Ax = 2.75
	Ay = 1.25
	Vy = 0.25
	omega = Vy/Ay
	
	#########################################################
	'''
	# straight line along x and sinusoidal along y
	x0 = x0i + 0.15*t
	y0 = y0i + Ay*math.sin(omega*t)
	'''
	#########################################################

	#########################################################
	# infinite shape trajectory
	x0 = x0i + Ax*math.sin(omega*t)
	y0 = y0i + Ay*math.sin(2*omega*t)
	#########################################################

	print(t,x0,y0)
	x0 = 5.544444561 + 4
	y0 = 5.544444561

	return x0,y0

# controller
def go2point(x_goal,y_goal):
	vc0 = 0
	wc0 = 0

	d = math.sqrt((x_goal-x)**2 + (y_goal-y)**2)
	'''	
	if(d>0.075):
		th0 = th0_
	else:
		th0 = math.atan2((y_goal-y),(x_goal-x))
	'''
	
	th0 = math.atan2((y_goal-y),(x_goal-x))
	alpha = th0 - th
	if(alpha<=-math.pi):
		alpha = alpha + 2.0*math.pi
	elif(alpha>math.pi):
		alpha = alpha - 2.0*math.pi

	if(d>0.075):
		kp_d = 0.75
		kp_th = 1.25*0
		vc0 = kp_d*d

		Ki = 0.35
		global Ith
		Ithmax = 0.8
		Ith = Ith + Ki*alpha*0.1
		if(abs(Ith)>Ithmax):
			Ith = (Ith/abs(Ith))*Ithmax

		wc0 = kp_th*alpha + Ith
		vc0max = 0.8
		wc0max = 1.5
		if(abs(vc0)>vc0max):
			vc0 = (vc0/abs(vc0))*vc0max
		if(abs(wc0)>wc0max):
			wc0 = (wc0/abs(wc0))*wc0max
	else:
		vc0 = 0
		wc0 = 0

	return vc0,wc0

# callback function
def odom_cb(msg, twist_pub):
	global x
	global y
	global th

	x   = msg.x
	y   = msg.y
	th = msg.theta
	vc_now  = msg.linear_velocity
	wc_now = msg.angular_velocity

if __name__ == '__main__':
	rospy.init_node('go_to_goal')
	twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber('/turtle1/pose', Pose, odom_cb, twist_pub)

	desired_twist = Twist()
	
	rate = rospy.Rate(10)
	t_start = rospy.get_time()

	while not rospy.is_shutdown():
		t = rospy.get_time() - t_start
		x0_goal,y0_goal = getx0y0(t)
		vc0,wc0 = go2point(x0_goal,y0_goal)

		desired_twist.linear.x  = vc0
		desired_twist.angular.z = wc0
		twist_pub.publish(desired_twist)

		rate.sleep()



