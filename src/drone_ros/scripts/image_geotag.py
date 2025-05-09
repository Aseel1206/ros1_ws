#!/usr/bin/env python

import rospy
import os
import csv
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import NavSatFix, Imu, Image
from cv_bridge import CvBridge

class SimpleKalman:
    def __init__(self, process_noise=1e-5, measurement_noise=1e-2):
        self.q = process_noise
        self.r = measurement_noise
        self.x = 0.0
        self.p = 1.0

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

class Geotagger:
    def __init__(self):
        rospy.init_node('geotagger_node', anonymous=True)

        self.gps_buffer = deque(maxlen=200)
        self.imu_buffer = deque(maxlen=300)
        self.image_buffer = deque(maxlen=1)

        self.kf_lat = SimpleKalman()
        self.kf_lon = SimpleKalman()
        self.kf_alt = SimpleKalman()
        self.kf_yaw = SimpleKalman()

        self.bridge = CvBridge()
        self.image_count = 0

        self.save_dir = '/home/edhitha/geotag_output'
        if not os.path.exists(self.save_dir):
	    os.makedirs(self.save_dir)

        self.csv_file = os.path.join(self.save_dir, 'geotag_data.csv')
        with open(self.csv_file, 'wb') as f:
	    writer = csv.writer(f, lineterminator='\n')
	    writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'orientation_w', 'yaw', 'filename'])


        #rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
	rospy.Subscriber('/usb_camera/image', Image, self.image_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        rospy.loginfo('ROS 1 Geotagger node started.')

    def gps_callback(self, msg):
        timestamp = msg.header.stamp.to_sec()
        self.gps_buffer.append((timestamp, msg))

    def imu_callback(self, msg):
        timestamp = rospy.Time.now().to_sec()
        self.imu_buffer.append((timestamp, msg))

    def image_callback(self, msg):
        image_time = msg.header.stamp.to_sec()
        rospy.loginfo("Image time: {}".format(image_time))
        rospy.loginfo("Buffer times: {}".format([t for t, _ in self.gps_buffer]))

        if len(self.image_buffer) > 0:
            self.process_images()

        self.image_buffer.clear()
        self.image_buffer.append((image_time, msg))

    def process_images(self):
        if len(self.image_buffer) == 0:
            return

        for image_time, image_msg in self.image_buffer:
            rospy.loginfo("Processing image with timestamp: {}".format(image_time))

            gps_interp = self.interpolate(self.gps_buffer, image_time, ['latitude', 'longitude', 'altitude'])
            quat = self.interpolate_quaternion(self.imu_buffer, image_time)

            if gps_interp is None or quat is None:
                rospy.logwarn("Interpolation failed, skipping frame.")
                continue

            lat = gps_interp['latitude']
            lon = gps_interp['longitude']
            alt = gps_interp['altitude']

            yaw = self.extract_yaw_from_quaternion(quat)
            filtered_yaw = self.kf_yaw.update(yaw)

            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                image_filename = os.path.join(self.save_dir, 'image_{:05d}.jpg'.format(self.image_count))
                cv2.imwrite(image_filename, cv_image)

                with open(self.csv_file, 'a') as f:
		    writer = csv.writer(f, lineterminator='\n')
		    writer.writerow([
			image_time,
			lat,
			lon,
			alt,
			quat[3],  # orientation_w (w)
			filtered_yaw,
			os.path.basename(image_filename)
		    ])


                rospy.loginfo("Saved image and data: {}".format(image_filename))
                self.image_count += 1

            except Exception as e:
                rospy.logerr("Image processing failed: {}".format(e))

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
        weight_upper = (target_time - lower[0]) / time_diff
        weight_lower = 1 - weight_upper

        q_lower_array = self.quaternion_to_array(q_lower)
        q_upper_array = self.quaternion_to_array(q_upper)
        q_interp = q_lower_array * weight_lower + q_upper_array * weight_upper

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

