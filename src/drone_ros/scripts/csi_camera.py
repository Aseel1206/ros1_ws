#!/usr/bin/env python
# -*- coding: utf-8 -*-

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import csv
import time
import datetime
import os
import threading 

# Import ROS specific libraries
import rospy
from std_msgs.msg import Time as RosTimeMsg 
from std_msgs.msg import UInt16 # Import for MAVROS mission current waypoint

# Import OpenCV and NumPy for image processing and saving
import cv2
import numpy as np

# --- Configuration ---
CSV_FILENAME = "python_absolute_timestamps_ros.csv"
FPS = 1.0 # Target frame rate
FRAME_SAVE_DIR = "captured_frames" # Directory to save image frames

# --- GStreamer & ROS Globals ---
first_argus_frame_system_time_ns = None
first_argus_frame_capture_time_ns = None
time_offset_ns = 0

frame_count = 0
pipeline = None
bus = None
gst_loop = None 

# ROS Publisher
timestamp_publisher = None

# Waypoint control variables
is_capturing = False # Flag to control if camera is actively capturing
current_waypoint_index = -1 # To store the latest waypoint received
start_waypoint_index = -1 # From ROS parameter
stop_waypoint_index = -1  # From ROS parameter

# CSV file handle
csv_file = None
csv_writer = None

# --- Functions to control GStreamer pipeline state ---
def start_capture():
    global is_capturing, pipeline, first_argus_frame_system_time_ns, first_argus_frame_capture_time_ns, time_offset_ns
    if not is_capturing:
        rospy.loginfo("Starting camera capture...")
        
        # Reset offset for fresh sync when starting capture again
        first_argus_frame_system_time_ns = None 
        first_argus_frame_capture_time_ns = None
        time_offset_ns = 0

        # Set pipeline to PLAYING state
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            rospy.logerr("Failed to set pipeline to PLAYING state.")
            return
        is_capturing = True
        rospy.loginfo("Camera capture STARTED.")
    else:
        rospy.loginfo("Camera is already capturing.")

def stop_capture():
    global is_capturing, pipeline
    if is_capturing:
        rospy.loginfo("Stopping camera capture...")
        # Set pipeline to NULL state (or PAUSED if you expect frequent starts/stops)
        ret = pipeline.set_state(Gst.State.NULL) 
        if ret == Gst.StateChangeReturn.FAILURE:
            rospy.logerr("Failed to set pipeline to NULL state.")
            return
        is_capturing = False
        rospy.loginfo("Camera capture STOPPED.")
    else:
        rospy.loginfo("Camera is not currently capturing.")

# --- MAVROS Waypoint Callback ---
def waypoint_callback(msg):
    global current_waypoint_index, start_waypoint_index, stop_waypoint_index

    new_waypoint_index = msg.data
    if new_waypoint_index != current_waypoint_index: # Only log/act on waypoint change
        current_waypoint_index = new_waypoint_index
        rospy.loginfo("Received new current waypoint: %d" % current_waypoint_index)

        if current_waypoint_index == start_waypoint_index:
            start_capture()
        elif current_waypoint_index == stop_waypoint_index:
            stop_capture()

def on_new_sample(sink):
    global frame_count, first_argus_frame_system_time_ns, \
           first_argus_frame_capture_time_ns, time_offset_ns, \
           timestamp_publisher, csv_writer, is_capturing # Added is_capturing to globals

    if not is_capturing:
        return Gst.FlowReturn.OK # Do nothing if not actively capturing

    sample = sink.emit("pull-sample")
    if sample:
        buffer = sample.get_buffer()
        argus_capture_time_ns = buffer.pts

        # If this is the first frame *since capture started*, establish the synchronization offset
        if first_argus_frame_system_time_ns is None:
            first_argus_frame_system_time_ns = int(time.time() * 1000000000)
            first_argus_frame_capture_time_ns = argus_capture_time_ns
            time_offset_ns = first_argus_frame_system_time_ns - first_argus_frame_capture_time_ns
            rospy.loginfo("Synchronization established (new capture session):")
            rospy.loginfo("  System time (ns) at first frame: %d" % first_argus_frame_system_time_ns)
            rospy.loginfo("  Argus time (ns) of first frame: %d" % first_argus_frame_capture_time_ns)
            rospy.loginfo("  Calculated offset (ns): %d" % time_offset_ns)

        absolute_timestamp_ns = argus_capture_time_ns + time_offset_ns
        
        # Option 1: Format as local time string for CSV/logging
        local_time_s = float(absolute_timestamp_ns) / 1000000000
        human_readable_time_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time_s)) + ".%06d" % (absolute_timestamp_ns % 1000000000 / 1000)
        
        frame_count += 1

        # --- Image Saving Logic ---
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value("width")
        height = s.get_value("height")

        success, mapinfo = buffer.map(Gst.MapFlags.READ)
        if not success:
            rospy.logerr("Error: Could not map buffer for frame %d" % frame_count)
            buffer.unmap(mapinfo)
            return Gst.FlowReturn.ERROR

        image_data = np.ndarray(
            (height, width, 4), 
            buffer=mapinfo.data,
            dtype=np.uint8
        )

        bgr_image = cv2.cvtColor(image_data, cv2.COLOR_RGBA2BGR)

        image_filename_local = os.path.join(FRAME_SAVE_DIR, "frame_%d.png" % frame_count)
        
        cv2.imwrite(image_filename_local, bgr_image)
        
        buffer.unmap(mapinfo)
        # --- End Image Saving Logic ---

        # --- ROS Publishing Logic (Timestamp only) ---
        ros_time_msg = RosTimeMsg()
        ros_time_msg.data = rospy.Time(0, absolute_timestamp_ns) 
        
        if timestamp_publisher:
            timestamp_publisher.publish(ros_time_msg)
            rospy.loginfo("Published timestamp for frame %d to /camera/timestamps" % frame_count)
        else:
            rospy.logwarn("Timestamp publisher not initialized!")
        # --- End ROS Publishing Logic ---

        # Log to CSV
        csv_writer.writerow([frame_count, argus_capture_time_ns, absolute_timestamp_ns, human_readable_time_str, os.path.basename(image_filename_local)])
        
        rospy.loginfo("Captured frame %d at:" % frame_count)
        rospy.loginfo("  Relative Argus (ns): %d" % argus_capture_time_ns)
        rospy.loginfo("  Absolute (ns):   %d" % absolute_timestamp_ns)
        rospy.loginfo("  Human Readable Time: %s" % human_readable_time_str)
        rospy.loginfo("  Saved to:            %s" % image_filename_local)
        rospy.loginfo("-" * 30)

    return Gst.FlowReturn.OK

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        rospy.loginfo("End-Of-Stream reached.")
        # We don't quit the GLib loop here as it needs to run continuously
        # to process state changes from the ROS thread.
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        rospy.logerr("GStreamer Error: %s from %s" % (err.message, message.src.get_name()))
        rospy.logerr("Debug: %s" % debug)
        # For critical errors, we might still want to quit the loop
        if gst_loop:
            gst_loop.quit()
    return True

def gst_thread_main():
    global pipeline, bus, gst_loop, csv_file, csv_writer

    Gst.init(None) # Initialize GStreamer here

    if not os.path.exists(FRAME_SAVE_DIR):
        os.makedirs(FRAME_SAVE_DIR)
        rospy.loginfo("Created directory: %s" % FRAME_SAVE_DIR)

    csv_file = open(CSV_FILENAME, 'wb') # 'wb' for binary write mode in Python 2
    csv_writer = csv.writer(csv_file)
    # Updated CSV header for clarity
    csv_writer.writerow(["FrameNumber", "Timestamp_nanoseconds_relative_camera", "Timestamp_nanoseconds_absolute_epoch", "Datetime_HumanReadable_Local", "Image_Filename"])

    try:
        pipeline_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! video/x-raw ! videorate ! video/x-raw,framerate=%d/1 ! nvvidconv ! video/x-raw,format=RGBA ! appsink name=mysink" % int(FPS)
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("mysink")
        if not appsink:
            raise RuntimeError("Could not get appsink element.")

        appsink.set_property("emit-signals", True)
        appsink.set_property("sync", False)
        appsink.connect("new-sample", on_new_sample)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_message)

        # Initially set pipeline to NULL state (stopped)
        # It will only start PLAYING when commanded by a waypoint
        ret = pipeline.set_state(Gst.State.NULL) 
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set pipeline to initial NULL state.")
        rospy.loginfo("GStreamer Pipeline initialized to NULL state. Awaiting capture command from waypoints.")

        gst_loop = GLib.MainLoop()
        gst_loop.run() # This loop runs continuously to process GStreamer events/bus messages

    except Exception as e:
        rospy.logerr("GStreamer thread error: %s" % e)
    finally:
        if pipeline:
            rospy.loginfo("Cleaning up GStreamer pipeline.")
            pipeline.set_state(Gst.State.NULL)
        if csv_file:
            csv_file.close()
            rospy.loginfo("CSV file '%s' closed." % CSV_FILENAME)


def main():
    global timestamp_publisher, start_waypoint_index, stop_waypoint_index

    rospy.init_node('camera_capture_node', anonymous=True)
    
    # Get parameters for start/stop waypoints
    # These MUST be set in your launch file
    start_waypoint_index = rospy.get_param('~start_waypoint_index')
    stop_waypoint_index = rospy.get_param('~stop_waypoint_index')

    rospy.loginfo("Configured START capture waypoint: %d" % start_waypoint_index)
    rospy.loginfo("Configured STOP capture waypoint: %d" % stop_waypoint_index)
    
    timestamp_publisher = rospy.Publisher('/camera/timestamps', RosTimeMsg, queue_size=1)
    
    # Subscribe to MAVROS current waypoint topic
    rospy.Subscriber('/mavros/mission/reached', UInt16, waypoint_callback)

    rospy.loginfo("Camera Capture ROS Node started. Publishing timestamps to /camera/timestamps.")
    rospy.loginfo("Subscribed to /mavros/mission/current for waypoint control.")
    rospy.loginfo("Press Ctrl+C to stop.")

    # Start the GStreamer pipeline thread. It will initialize the pipeline
    # but not start capturing until commanded by a waypoint.
    gst_thread = threading.Thread(target=gst_thread_main)
    gst_thread.daemon = True 
    gst_thread.start()
    print("start point: " +start_waypoint_index)
    if start_waypoint_index == -1:
	start_capture()

    # Keep the ROS node alive
    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass 

    # On ROS shutdown, signal GStreamer thread to quit
    if gst_loop: 
        rospy.loginfo("ROS shutdown detected. Signaling GStreamer loop to quit.")
        gst_loop.quit()
    
    gst_thread.join()
    rospy.loginfo("Camera Capture ROS Node stopped.")

if __name__ == '__main__':
    main()
