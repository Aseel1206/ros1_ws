#!/usr/bin/env python

import rospy
import os
import csv
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import NavSatFix, Imu, Image
from cv_bridge import CvBridge
from std_msgs.msg import String


class Geotagger:
    def __init__(self):
        rospy.init_node('geotagger_node', anonymous=True)

        self.gps_buffer = deque(maxlen=200)
        self.imu_buffer = deque(maxlen=300)
        self.image_buffer = deque(maxlen=1)

        self.bridge = CvBridge()
        self.image_count = 0

        self.save_dir = '/home/edhitha/geotag_output'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.csv_file = os.path.join(self.save_dir, 'geotag_data.csv')
        with open(self.csv_file, 'wb') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'orientation_w', 'yaw', 'filename'])

        rospy.Subscriber('/camera/timestamps', String, self.image_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.status_pub = rospy.Publisher('/geotagger/status', String, queue_size=10, latch=True)

        rospy.loginfo('ROS 1 Geotagger node started.')
        self.status_pub.publish("active")

    def gps_callback(self, msg):
        timestamp = msg.header.stamp.to_sec()
        self.gps_buffer.append((timestamp, msg))

    def imu_callback(self, msg):
        timestamp = rospy.Time.now().to_sec()
        self.imu_buffer.append((timestamp, msg))

    def image_callback(self, msg):
        try:
            timestamp_str, filename = msg.data.split(',')
            image_time = float(timestamp_str)
            self.image_buffer.clear()
            self.image_buffer.append((image_time, filename))
            rospy.loginfo("Received image metadata: time={}, file={}".format(image_time, filename))
            self.process_images()
        except Exception as e:
            rospy.logerr("Failed to parse image metadata: {}".format(e))

    def process_images(self):
        if len(self.image_buffer) == 0:
            return

        for image_time, image_filename in self.image_buffer:
            rospy.loginfo("Processing metadata with timestamp: {}".format(image_time))

            gps_interp = self.interpolate(self.gps_buffer, image_time, ['latitude', 'longitude', 'altitude'])
            quat = self.interpolate_quaternion(self.imu_buffer, image_time)

            if gps_interp is None or quat is None:
                self.status_pub.publish("interpolation failed")
                rospy.logwarn("Interpolation failed, skipping frame.")
                continue

            lat = gps_interp['latitude']
            lon = gps_interp['longitude']
            alt = gps_interp['altitude']

            yaw = self.extract_yaw_from_quaternion(quat)
            # self.kf_yaw.update() is missing in your code. If unused, remove this.
            filtered_yaw = yaw  # Placeholder

            try:
                with open(self.csv_file, 'ab') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        image_time,
                        lat,
                        lon,
                        alt,
                        quat[3],  # orientation_w
                        filtered_yaw,
                        image_filename
                    ])

                rospy.loginfo("Saved metadata for file: {}".format(image_filename))
                self.status_pub.publish("geotagging")
                self.image_count += 1

            except Exception as e:
                rospy.logerr("Failed to save metadata: {}".format(e))

    def interpolate(self, buffer, target_time, fields):
        lower = None
        upper = None

        for i in range(len(buffer)-1):
            if buffer[i][0] <= target_time < buffer[i+1][0]:
                lower = buffer[i]
                upper = buffer[i+1]
                break

        if lower is None or upper is None:
            return None

        interpolated_data = {}
        for field in fields:
            lower_value = getattr(lower[1], field)
            upper_value = getattr(upper[1], field)
            time_diff = upper[0] - lower[0]
            if time_diff == 0:
                rospy.logwarn("Zero time difference in interpolation.")
                return None
            weight_upper = (target_time - lower[0]) / time_diff
            weight_lower = 1 - weight_upper
            interpolated_data[field] = lower_value * weight_lower + upper_value * weight_upper

        return interpolated_data

    def interpolate_quaternion(self, buffer, target_time):
        lower = None
        upper = None

        for i in range(len(buffer)-1):
            if buffer[i][0] <= target_time < buffer[i+1][0]:
                lower = buffer[i]
                upper = buffer[i+1]
                break

        if lower is None or upper is None:
            return None

        q_lower = lower[1].orientation
        q_upper = upper[1].orientation
        time_diff = upper[0] - lower[0]
        if time_diff == 0:
            return None
        weight_upper = (target_time - lower[0]) / time_diff
        weight_lower = 1 - weight_upper

        q_lower_array = self.quaternion_to_array(q_lower)
        q_upper_array = self.quaternion_to_array(q_upper)
        q_interp = np.multiply(q_lower_array, weight_lower) + np.multiply(q_upper_array, weight_upper)

        return q_interp

    def quaternion_to_array(self, q):
        return np.array([q.x, q.y, q.z, q.w])

    def extract_yaw_from_quaternion(self, quat):
        x, y, z, w = quat
        _, _, yaw = self.euler_from_quaternion([x, y, z, w])
        return yaw

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat
        roll_x = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_y = np.arcsin(2.0 * (w * y - z * x))
        yaw_z = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll_x, pitch_y, yaw_z

if __name__ == '__main__':
    try:
        geotagger = Geotagger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

