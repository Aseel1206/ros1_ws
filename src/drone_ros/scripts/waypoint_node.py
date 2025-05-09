#!/usr/bin/env python

import rospy
import time
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandTOL, WaypointSetCurrent
from mavros_msgs.msg import Waypoint

class MissionUploaderFromFile:
    def __init__(self):
        rospy.init_node('mission_uploader_from_file', anonymous=True)
        
        self.waypoint_file = rospy.get_param('~mission_file', '/home/edhitha/SUAS_24.waypoints')
        rospy.loginfo("Reading mission from file: {}".format(self.waypoint_file))
        
        waypoints = self.parse_waypoint_file(self.waypoint_file)
        if not waypoints:
            rospy.logerr("No waypoints found.")
            return

        self.push_mission(waypoints)
        self.set_mode("GUIDED")
        time.sleep(1)
        self.arm(True)
        time.sleep(1)

        self.start_mission()

    def parse_waypoint_file(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            lines = file.readlines()

        if not lines or not lines[0].startswith("QGC WPL"):
            rospy.logerr("Invalid or empty waypoint file.")
            return []

        for line in lines[1:]:
            parts = line.strip().split('\t')
            if len(parts) < 12:
                continue
            wp = Waypoint()
            wp.is_current = bool(int(parts[1]))
            wp.frame = int(parts[2])
            wp.command = int(parts[3])
            wp.param1 = float(parts[4])
            wp.param2 = float(parts[5])
            wp.param3 = float(parts[6])
            wp.param4 = float(parts[7])
            wp.x_lat = float(parts[8])
            wp.y_long = float(parts[9])
            wp.z_alt = float(parts[10])
            wp.autocontinue = bool(int(parts[11]))
            waypoints.append(wp)

        rospy.loginfo("Loaded {} waypoints from file.".format(len(waypoints)))
        return waypoints

    def push_mission(self, waypoints):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            resp = push_srv(0, waypoints)
            if resp.success:
                rospy.loginfo("Pushed {} waypoints successfully.".format(resp.wp_transfered))
            else:
                rospy.logerr("Failed to push waypoints.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def set_mode(self, mode_str):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            resp = mode_srv(0, mode_str)
            rospy.loginfo("Set mode to {}".format(mode_str))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def arm(self, value=True):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            resp = arm_srv(value)
            if resp.success:
                rospy.loginfo("Vehicle armed.")
            else:
                rospy.logerr("Failed to arm.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def start_mission(self):
        self.takeoff(altitude=10.0)
        self.set_mode("AUTO")

        rospy.wait_for_service('/mavros/mission/set_current')
        try:
            set_wp_srv = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
            resp = set_wp_srv(0)
            if resp.success:
                rospy.loginfo("Mission started from waypoint 0.")
            else:
                rospy.logerr("Failed to start mission.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def takeoff(self, altitude=10.0):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            resp = takeoff_srv(0.0, 0.0, 0.0, 0.0, altitude)
            if resp.success:
                rospy.loginfo(" Takeoff command sent, altitude: {}m".format(altitude))
            else:
                rospy.logerr("Takeoff failed.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))


if __name__ == '__main__':
    try:
        MissionUploaderFromFile()
    except rospy.ROSInterruptException:
        pass

