#!/usr/bin/env python
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    br.sendTransform()