#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State
from std_msgs.msg import String
from sensor_msgs.msg import TimeReference
import json


class DroneStatusMonitor:
    def __init__(self):
        rospy.init_node('drone_status_monitor', anonymous=True)

        self.mavros_connected = False
        self.imu_available = False
        self.gps_available = False
	self.geotagger_status = "unknown"
	self.camera_status = "Initializing..."

        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)
	rospy.Subscriber('/geotagger/status', String, self.geotagger_status_cb)
	rospy.Subscriber('/mavros/time_reference', TimeReference, self.time_reference_cb)
	rospy.Subscriber('/camera/status', String, self.camera_status_cb)
	self.status_pub = rospy.Publisher('/drone_status', String, queue_size=10)

	self.fcu_time = None

        rospy.Timer(rospy.Duration(1), self.status_check)

    def state_cb(self, msg):
        self.mavros_connected = msg.connected

    def time_reference_cb(self, msg):
        self.fcu_time = msg.time_ref  # This is a rospy.Time object

    def imu_cb(self, msg):
        self.imu_available = True

    def gps_cb(self, msg):
        if msg.status.status >= 0:
        	self.gps_available = True
	else:
		self.gps_available = False

    def geotagger_status_cb(self, msg):
        self.geotagger_status = msg.data

    def camera_status_cb(self, msg):
        self.camera_status = msg.data

    def status_check(self, event):
        try:
            running_nodes = rosnode.get_node_names()
            mavros_running = any('mavros' in node for node in running_nodes)
	    geotagger_running = any('image_geotag' in node or 'geotagger_node' in node for node in running_nodes)
	    camera_node_running = any('camera_capture_node' in node for node in running_nodes) 
        except:
            mavros_running = False
	    geotagger_running = False
	    camera_node_running = False
	ros_time = rospy.Time.now()
	ros_time_str = ros_time.to_sec()

	fcu_time_str = "N/A"
	if self.fcu_time:
	    fcu_time_str = self.fcu_time.to_sec()
	status_dict = {
		"ros_time_sec": ros_time_str,
		"fcu_time_sec": fcu_time_str,
		"mavros_running": mavros_running,
		"fcu_connected": self.mavros_connected,
		"imu_available": self.imu_available,
		"gps_available": self.gps_available,
		"geotagger_running": geotagger_running,
		"geotagger_status": self.geotagger_status,
		"camera_node_running": camera_node_running,
		"camera_status": self.camera_status
	    }
	self.status_pub.publish(json.dumps(status_dict))

        print("\n" + "="*30) # Add a separator for better readability
        print("[DRONE STATUS MONITOR]")
        print("="*30)
        print("ROS Time (sec):          {:.9f}".format(ros_time_str))
        print("FCU Time (sec):          {}".format(fcu_time_str))
        print("-" * 30)
        print("MAVROS node running:     {}".format("running" if mavros_running else "NOT RUNNING"))
        print("FCU connected:           {}".format("CONNECTED" if self.mavros_connected else "NOT CONNECTED"))
        print("IMU data received:       {}".format("data available" if self.imu_available else "NO DATA"))
        print("GPS data received:       {}".format("data available" if self.gps_available else "NO FIX"))
        print("-" * 30)
        print("Image Geotagger running: {}".format("running" if geotagger_running else "NOT RUNNING"))
        print("Image Geotagger status:  {}".format(self.geotagger_status))
        print("-" * 30)
        print("Camera Node running:     {}".format("running" if camera_node_running else "NOT RUNNING")) # NEW: Display camera node status
        print("Camera Status:           {}".format(self.camera_status)) # NEW: Display camera status messages
        print("="*30 + "\n")

if __name__ == '__main__':
    try:
        monitor = DroneStatusMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

