#!/usr/bin/env python

import rospy
import time

def print_timestamp():
    rospy.init_node('timestamp_printer', anonymous=True)
    rate = rospy.Rate(1)  # 10 Hz

    while not rospy.is_shutdown():
        # Get precise system time
        current_time = time.time()
        # Format with full precision
        rospy.loginfo("System Time: %.9f seconds" % current_time)
        rate.sleep()

if __name__ == '__main__':
    try:
        print_timestamp()
    except rospy.ROSInterruptException:
        pass

