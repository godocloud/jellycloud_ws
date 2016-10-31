#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
import serial
import thread
import time
from sensor_msgs.msg import LaserScan
import struct
import string
import binascii
from std_msgs.msg import String


class MSerialPort:
    message = ''

    def __init__(self,port,buand):
        self.port=serial.Serial(port,buand)
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self,data):
        number = self.port.write(data)
        return number

    def read_data(self):
        is_begin_shipping = False  # 标记是否开始装填数据
        buf = ''  # 缓冲区
        ship_buf = ''  # 装填区
        begin_position = -1
        while True:
            data = self.port.read(1811)
            buf = buf + data;
            if len(buf) <= 7:
                continue
            for i in range(len(buf)-6):
                if ord(buf[i]) == 0xA5 and ord(buf[i+6]) == 0x81:
                    begin_position = i
                    break

            if begin_position == -1:
                if is_begin_shipping:
                    ship_buf += buf
                else:
                    # 丢弃和雷达数据无关的数据
                    print "read useless data"
            else:
                if is_begin_shipping:
                    ship_buf += buf[0:begin_position]
                    # 处理 ship_buf
                    # todo
                    laser = LaserScan()
                    laser.header.stamp = rospy.Time.now()
                    laser.header.frame_id = "laser_frame"
                    laser.angle_min = 0
                    laser.angle_max = 0
                    laser.angle_increment = 3.14 / 358
                    laser.time_increment = 0
                    laser.range_min = 0.0
                    laser.range_max = 6.
                    begin_catch = False
                    laser_data = ship_buf[7:]
                    for i in range(len(laser_data)/5):
                        if ord(laser_data[i*5]) == 0x00:
                            if begin_catch:
                                laser.ranges.append(struct.unpack('<h', laser_data[i * 5 + 3] + laser_data[i * 5 + 4])[0]/1000.)
                        if ord(laser_data[i*5]) == 0x01:
                            if not begin_catch:
                                begin_catch = True
                    print laser.ranges

                    pub = rospy.Publisher('scan', LaserScan, queue_size=50)
                    pub.publish(laser)
                    #print len(ship_buf)
                    #print ship_buf.encode("hex")
                    ship_buf = buf[begin_position:]
                else:
                    is_begin_shipping = True
                    ship_buf = buf[begin_position:]
            buf = ''


if __name__ == '__main__':
    rospy.init_node('lidar', anonymous=True)
    # ser = serial.Serial('/dev/ttyUSB0', 115200)
    mSerial = MSerialPort('/dev/ttyUSB0', 115200)
    thread.start_new_thread(mSerial.read_data, ())

    mSerial.send_data(bytearray.fromhex('A5 2C D1'))
    mSerial.send_data(bytearray.fromhex('A5 20 C7'))
    time.sleep(1)
    rospy.spin()
    # data = ''
    # while ser.inWaiting() > 0:
    #     data += ser.read(1)
    # if data != '':
    #     # print data.encode("hex")
    #     data_hex = data.encode("hex")
    #     print data_hex
