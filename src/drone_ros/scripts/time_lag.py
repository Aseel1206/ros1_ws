#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    ros_time = rospy.Time.now()
    msg_time = msg.header.stamp
    delay = (ros_time - msg_time).to_sec()
    rospy.loginfo("IMU delay: %.6f seconds", delay)

rospy.init_node('latency_checker')
rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
rospy.spin()

