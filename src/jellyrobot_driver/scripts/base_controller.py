#!/usr/bin/env python
# -*- coding: utf-8 -*-
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
import tf.transformations
import serial
import sys
from geometry_msgs.msg import Twist
D = 0.16

try:  
    ser = serial.Serial('/dev/ttyUSB1', 115200)
except Exception, e:  
    print 'open serial failed.'  
    exit(1) 

def callback(msg ):

	cmd_twist_rotation =  msg.angular.z 
	cmd_twist_x  = msg.linear.x 
#	cmd_twist_y =  msg.linear.y #这个一般为0

#将twist消息转化为左右轮各自的期望速度
	wheelspeed = odom_to_speed(cmd_twist_x, cmd_twist_rotation)        
	print 'msg:', msg                #打印得到的twist消息
	print wheelspeed                 #打印转化后的速度 

#蓝牙串口发送到DSP  wheelspeed[0]左轮速度， wheelspeed[1]右轮速度, not ok yet        
	serial_send(wheelspeed[0], wheelspeed[1]) 
	

def odom_to_speed(cmd_twist_x =0, cmd_twist_rotation=0):
#'一般情况下，linear_y = 0 所以只需关注twist.linear.x 和 twist.angle.z的转换'
	cent_speed = cmd_twist_x        #前进的速度，即两轮的中心速度

#将 指定的转速twist.angular.z 转化为左右轮的差速
	yawrate2 = yawrate_to_speed(cmd_twist_rotation)   

	Lwheelspeed = cent_speed - yawrate2
	Rwheelspeed = cent_speed + yawrate2

	return Lwheelspeed, Rwheelspeed

def yawrate_to_speed(yawrate):
	if yawrate >= 0:
		theta_to_speed = yawrate * (D/2) #CCW
	else:
		theta_to_speed = yawrate * (D/2) #CW
	#print "*************theta_to_speed = ", theta_to_speed

#yawrate ：rad/s   
	x = theta_to_speed   
	return   x

def serial_send(Lwheelspeed, Rwheelspeed):
	if Lwheelspeed<0:
		left_dir=0
		left_speed=int(-Lwheelspeed*100)# 0.2m/s * 100=20cm/s in real robot
	else:
		left_dir=1
		left_speed=int(Lwheelspeed*100)
	
	if Rwheelspeed<0:
		right_dir=0
		right_speed=int(-Rwheelspeed*100)
	else:
		right_dir=1
		right_speed=int(Rwheelspeed*100)

	check_sum = (0xfe+0x04+0x01+left_dir+left_speed+right_dir+right_speed)%256
	
	cmd_head = "fffe040100"
	cmd_l_dir = "%02x" % left_dir
	cmd_l_speed = "%02x" % left_speed
	cmd_r_dir = "%02x" % right_dir
	cmd_r_speed = "%02x" % right_speed
	cmd_sum = "%02x" % check_sum
	cmd_send = cmd_head+cmd_l_dir+cmd_l_speed+cmd_r_dir+cmd_r_speed+cmd_sum
	#print cmd_send


	hex_cmd_send = cmd_send.decode("hex")
	s = ser.write(hex_cmd_send)


def listener():
    rospy.init_node('base_controller')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)#/cmd_vel
    rospy.spin()
    
if __name__ == '__main__':
    listener()



  



    
