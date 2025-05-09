#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

def arm_and_takeoff(target_alt=10.0):
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/takeoff')

    try:
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        # Set mode to GUIDED
        rospy.loginfo("Setting mode to GUIDED...")
        mode_resp = set_mode_service(0, 'GUIDED')
        rospy.loginfo("Mode set: {}".format(mode_resp.mode_sent))

        time.sleep(2)

        # Arm the drone
        rospy.loginfo("Arming the drone...")
        arm_resp = arm_service(True)
        rospy.loginfo("Arm result: {}".format(arm_resp.success))

        time.sleep(2)

        # Takeoff
        rospy.loginfo("Initiating takeoff...")
        takeoff_resp = takeoff_service(
            altitude=target_alt,
            latitude=0.0,
            longitude=0.0,
            min_pitch=0.0,
            yaw=0.0
        )
        rospy.loginfo("Takeoff result: {}".format(takeoff_resp.success))

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {e}")

def main():
    rospy.init_node('arm_takeoff_node')
    arm_and_takeoff(10.0)

if __name__ == '__main__':
    main()

