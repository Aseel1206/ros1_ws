#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MockImagePublisher:
    def __init__(self):
        rospy.init_node('mock_image_publisher', anonymous=True)

        # Create publisher
        self.image_pub = rospy.Publisher('/image_raw', Image, queue_size=10)

        # CvBridge to convert images
        self.bridge = CvBridge()

        # Timer at 1 Hz
        rospy.Timer(rospy.Duration(1), self.publish_mock_image)

    def publish_mock_image(self, event):
        # Create a mock black image
        mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(mock_image, 'Mock Image', (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255, 255, 255), 2, cv2.LINE_AA)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(mock_image, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            self.image_pub.publish(ros_image)
            rospy.loginfo("Published a mock image")
        except Exception as e:
            rospy.logerr("Error in publishing mock image: {}".format(e))

if __name__ == '__main__':
    try:
        MockImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

