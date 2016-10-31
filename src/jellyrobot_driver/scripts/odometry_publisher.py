#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin, cos, pi
import serial
import sys
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

left_dir_buf=0
left_speed_buf=0.0
right_dir_buf=0
right_speed_buf=0.0
left_speed=0.0
right_speed=0.0

D = 0.16
try:  
    ser = serial.Serial('/dev/ttyUSB1', 115200)
except Exception, e:  
    print 'open serial failed.'  
    exit(1) 

def serial_read():
	data = ''
	while ser.inWaiting() > 0:
		data += ser.read(1)
	if data != '':
		#print data.encode("hex")
		data_hex = data.encode("hex")
		data_process(data_hex);
		#print data_hex[:2],data_hex[2:4],data_hex[4:6],data_hex[6:8],data_hex[-2:]

def data_process(data_buf):
	if (data_buf[:2]=="ff" and data_buf[2:4]=="fe"):
		data_length = int(data_buf[4:6],16)
		check_sum = int(data_buf[-2:],16)
		#print "data_length=",data_length
		data_sum = 0
		for i in range(data_length):
			data_sum += int(data_buf[10+2*i:12+2*i],16)
			#print "data_sum =",hex(data_sum)
		check_sum_temp = (0xfe + data_length + 0x81 + data_sum)%256
		#print "data_sum =",data_sum
		#print "check sum temp =",check_sum_temp
		if(check_sum != check_sum_temp):
			return 0
		print "data check ok"

		if(data_buf[6:8] == "01" and data_buf[8:10] == "80"):#speed now
			global left_dir_buf, left_speed_buf, right_dir_buf, right_speed_buf
			left_dir_buf = int(data_buf[10:12],16)
			left_speed_buf = int(data_buf[12:14],16)
			right_dir_buf = int(data_buf[14:16],16)
			right_speed_buf = int(data_buf[16:18],16)
		#print left_dir_buf, left_speed_buf, right_dir_buf, right_speed_buf

	
def speed_linear():
	global left_speed, right_speed
	if(left_dir_buf >= 0):
		left_speed = left_speed_buf/100.0
	else:
		left_speed = (-left_speed_buf)/100.0
	#print "left_speed=",left_speed
	if(right_dir_buf >= 0):
		right_speed = right_speed_buf/100.0
	else:
		right_speed = (-right_speed_buf)/100.0
	#print "right_speed=",right_speed
	#print "(left_speed + right_speed)/2", (left_speed + right_speed)/2
	return (left_speed + right_speed)/2

def speed_angular():
	#print "(right_speed - left_speed)/D", (right_speed - left_speed)/D
	return (right_speed - left_speed)/D

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0


current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
	#we read serial data at the beginning
    serial_read()
    vx = speed_linear()
    vy = 0.0
    vth = speed_angular()
    #print "vx=",vx
    #print "vth=",vth
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
