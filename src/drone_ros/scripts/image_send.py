#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def capture_and_publish():
    # Initialize the ROS Node
    rospy.init_node('usb_camera_publisher', anonymous=True)
    
    # Create a publisher that will publish the captured images
    image_pub = rospy.Publisher('/usb_camera/image', Image, queue_size=10)
    
    # Create a CVBridge instance to convert OpenCV images to ROS images
    bridge = CvBridge()
    
    # Open the USB camera (device 0 is typically the default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return
    
    rospy.loginfo("USB Camera successfully opened")

    # Set the loop rate to 1 Hz (1 frame per second)
    rate = rospy.Rate(1)  # 1 Hz = 1 frame per second

    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue
        
        # Get the current timestamp
        timestamp = rospy.Time.now()
        
        # Convert the OpenCV image to a ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Add the timestamp to the ROS Image message
        ros_image.header.stamp = timestamp
        
        # Publish the image to the topic
        image_pub.publish(ros_image)
        
        rospy.loginfo("Published image at time: %s", timestamp)
        
        # Sleep to maintain the desired loop rate (1 frame per second)
        rate.sleep()
    
    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        capture_and_publish()
    except rospy.ROSInterruptException:
        pass

