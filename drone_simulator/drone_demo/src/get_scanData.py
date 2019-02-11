#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
	print 'Value at 0 degrees:'
	print msg.ranges[0]
	print 'Value at 70 degrees: '
	print msg.ranges[400]
	print 'Value at 90 degrees:'
	print msg.ranges[540]
	print 'Value at 110 degrees: '
	print msg.ranges[680]
	print 'Value at 180 degrees:'
	print msg.ranges[1080]

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
