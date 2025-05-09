#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointReached
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from threading import Timer
from datetime import datetime
import piexif
from PIL import Image
import os
import cv2
import csv


class WaypointMonitor(object):
    def __init__(self):
        rospy.init_node('waypoint_monitor')

        self.waypoint_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.reached_callback)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.alt_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.alt_callback)
        self.heading_sub = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.heading_callback)

        self.image_source_path = '/home/edhitha/demo.png'  # You can ignore this for camera-based capture
        self.captured_folder = '/home/edhitha/captured'
        if not os.path.exists(self.captured_folder):
    		os.makedirs(self.captured_folder)

	self.csv_file_path = os.path.join(self.captured_folder, 'geotagging.csv')
	if not os.path.exists(self.csv_file_path):
	    with open(self.csv_file_path, mode='w') as f:
		writer = csv.writer(f)
		writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Altitude', 'Heading', 'Filename'])


        self.saving_images = False
        self.image_copy_timer = None

        self.gps_timestamp = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.heading = None
	self.start_waypoint = 2
	self.stop_waypoint =3

        rospy.loginfo("Waypoint Monitor Node started. Waiting for waypoints...")

        # Camera initialization
        self.cap = cv2.VideoCapture(0)  # Use the correct camera index for Jetson
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera.")
            exit(1)

    def imu_callback(self, msg):
        pass  # Optional: process IMU data if needed

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        stamp = msg.header.stamp
        self.gps_timestamp = stamp.secs + stamp.nsecs * 1e-9

    def alt_callback(self, msg):
        self.altitude = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def reached_callback(self, msg):
        rospy.loginfo("Waypoint {} reached.".format(msg.wp_seq))

        if msg.wp_seq == self.start_waypoint:
            rospy.loginfo("Start capturing images.")
            self.saving_images = True
            self.start_image_capture_loop()

        elif msg.wp_seq == self.stop_waypoint:
            rospy.loginfo("Stop capturing images.")
            self.saving_images = False
            if self.image_copy_timer:
                self.image_copy_timer.cancel()

    def start_image_capture_loop(self):
        if not self.saving_images:
            return

        self.capture_image()
        self.image_copy_timer = Timer(1.0, self.start_image_capture_loop)
        self.image_copy_timer.start()

    def capture_image(self):
        if not self.cap.isOpened():
            rospy.logwarn("Camera is not available.")
            return

        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image.")
            return

        if self.gps_timestamp is None:
            rospy.logwarn("No GPS timestamp available yet.")
            return

        dt = datetime.fromtimestamp(self.gps_timestamp)
        timestamp_str = dt.strftime('%Y%m%d_%H%M%S_%f')
        dest_path = os.path.join(self.captured_folder, 'image_{}.jpg'.format(timestamp_str))

        rospy.loginfo("Latitude: {} Longitude: {} Altitude: {}".format(self.latitude, self.longitude, self.altitude))

        try:
            # Save the captured frame as JPEG
            cv2.imwrite(dest_path, frame)

            # Add GPS metadata to the image
            if all(v is not None for v in [self.latitude, self.longitude, self.altitude, self.heading]):
                def to_deg(value):
                    deg = int(value)
                    min_ = int((value - deg) * 60)
                    sec = int(((value - deg - min_ / 60) * 3600) * 100)
                    return ((deg, 1), (min_, 1), (sec, 100))

                gps_ifd = {
                    piexif.GPSIFD.GPSLatitudeRef: b'N' if self.latitude >= 0 else b'S',
                    piexif.GPSIFD.GPSLatitude: to_deg(abs(self.latitude)),
                    piexif.GPSIFD.GPSLongitudeRef: b'E' if self.longitude >= 0 else b'W',
                    piexif.GPSIFD.GPSLongitude: to_deg(abs(self.longitude)),
                    piexif.GPSIFD.GPSAltitudeRef: 0,
                    piexif.GPSIFD.GPSAltitude: (int(self.altitude * 100), 100),
                    piexif.GPSIFD.GPSImgDirectionRef: b'T',
                    piexif.GPSIFD.GPSImgDirection: (int(self.heading * 100), 100),
                }

                exif_dict = {"GPS": gps_ifd}
                exif_bytes = piexif.dump(exif_dict)

                piexif.insert(exif_bytes, dest_path)
		with open(self.csv_file_path, mode='a') as f:
		    writer = csv.writer(f)
		    writer.writerow([timestamp_str, self.latitude, self.longitude, self.altitude, self.heading, dest_path])

                rospy.loginfo("Metadata added to {}".format(dest_path))
            else:
                rospy.logwarn("Incomplete GPS/heading data. Metadata not added.")

            rospy.loginfo("Image saved: {}".format(dest_path))
        except Exception as e:
            rospy.logerr("Failed to capture image: {}".format(str(e)))

    def shutdown(self):
        if self.cap.isOpened():
            self.cap.release()

def main():
    waypoint_monitor = None  # Declare it before the try block
    try:
        waypoint_monitor = WaypointMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if waypoint_monitor:
            waypoint_monitor.shutdown()

if __name__ == '__main__':
    main()

