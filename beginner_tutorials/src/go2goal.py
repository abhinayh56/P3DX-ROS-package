#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# desired states in inertial frame
x0 = 5.544444561 - 2
y0 = 5.544444561 + 0.001
th0_ = 0*math.pi/180.0
vx0 = 0
vy0 = 0
vth0 = 0

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

# vehicle command velocity
global vc
global wc
vc = 0
wc = 0

# controller
def go2point(x0,y0):
	kw = 0
	kx = 0
	ky = 0
	
	global vc
	global wc

	d = math.sqrt((x0-x)**2 + (y0-y)**2)
	'''	
	if(d>0.075):
		th0 = th0_
	else:
		th0 = math.atan2((y0-y),(x0-x))
	'''
	
	th0 = math.atan2((y0-y),(x0-x))
	alpha = th0 - th
	if(alpha<=-math.pi):
		alpha = alpha + 2.0*math.pi
	elif(alpha>math.pi):
		alpha = alpha - 2.0*math.pi

	if(d>0.075):
		kp_d = 1
		kp_th = 1
		vc = kp_d*d
		wc = kp_th*alpha
		if(abs(vc)>0.8):
			vc = (vc/abs(vc))*0.8
		if(abs(wc)>1):
			wc = (wc/abs(wc))*1
	else:
		vc = 0
		wc = 0

# callback function
def odom_cb(msg, twist_pub):
	global x
	global y
	global th
	global vx
	global vth

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
	while not rospy.is_shutdown():
		vc = 0
		wc = 0

		go2point(x0,y0)


		desired_twist.linear.x  = vc
		desired_twist.angular.z = wc
		twist_pub.publish(desired_twist)

		rate.sleep()



