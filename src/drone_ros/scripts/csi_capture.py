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
import subprocess # Add this at the top of your script
import os         # Add this at the top of your script
import signal     # Add this at the top of your script
# Import ROS specific libraries
import rospy
from std_msgs.msg import Time as RosTimeMsg 
from mavros_msgs.msg import WaypointReached 
from std_msgs.msg import String 

# Import OpenCV and NumPy for image processing and saving
import cv2
import numpy as np

# --- Configuration ---
CSV_FILENAME = "/home/edhitha/python_absolute_timestamps_ros.csv"
FPS = 1.0 # Target frame rate
FRAME_SAVE_DIR = "/home/edhitha/captured_frames" # Directory to save image frames

# --- GStreamer & ROS Globals ---
first_argus_frame_system_time_ns = None
first_argus_frame_capture_time_ns = None
time_offset_ns = 0

frame_count = 0
pipeline = None
bus = None
gst_loop = None 

# ROS Publishers
timestamp_publisher = None
status_publisher = None # NEW: Global variable for status publisher

# Waypoint control variables
is_capturing = False # Flag to control if camera is actively capturing
current_waypoint_index = -1 # To store the latest waypoint received
start_waypoint_index = -1 # From ROS parameter
stop_waypoint_index = -1  # From ROS parameter

# CSV file handle
csv_file = None
csv_writer = None

# NEW: Helper function to publish status messages
def publish_status(message):
    global status_publisher
    if status_publisher:
        status_msg = String()
        status_msg.data = message
        status_publisher.publish(status_msg)
    rospy.loginfo("[STATUS_PUB] %s" % message) # Keep logging as well for local debugging

# --- Functions to control GStreamer pipeline state ---
def start_capture():
    global is_capturing, pipeline, first_argus_frame_system_time_ns, first_argus_frame_capture_time_ns, time_offset_ns
    if not is_capturing:
        if pipeline is None:
            publish_status("ERROR: Cannot start capture, GStreamer pipeline is not initialized yet.")
            return

        publish_status("Attempting to START camera capture...")
        
        # Reset offset for fresh sync when starting capture again
        first_argus_frame_system_time_ns = None 
        first_argus_frame_capture_time_ns = None
        time_offset_ns = 0

        # Set pipeline to PLAYING state
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            publish_status("ERROR: Failed to set pipeline to PLAYING state. Check GStreamer errors.")
            return
        is_capturing = True
        publish_status("Camera capture **STARTED**.")
    else:
        publish_status("Camera is already capturing.")

def stop_capture():
    global is_capturing, pipeline
    if is_capturing:
        if pipeline is None:
            publish_status("WARNING: Cannot stop capture, GStreamer pipeline is not initialized, but capture flag is set.")
            is_capturing = False # Correct the flag in case of inconsistency
            return

        publish_status("Attempting to STOP camera capture...")
        # Set pipeline to NULL state (or PAUSED if you expect frequent starts/stops)
        ret = pipeline.set_state(Gst.State.NULL) 
        if ret == Gst.StateChangeReturn.FAILURE:
            publish_status("ERROR: Failed to set pipeline to NULL state. Check GStreamer errors.")
            return
        is_capturing = False
        publish_status("Camera capture **STOPPED**.")
    else:
        publish_status("Camera is not currently capturing.")

# --- MAVROS Waypoint Callback ---
def waypoint_callback(msg):
    global current_waypoint_index, start_waypoint_index, stop_waypoint_index

    new_waypoint_index = msg.data
    if new_waypoint_index != current_waypoint_index: # Only log/act on waypoint change
        old_waypoint_index = current_waypoint_index
        current_waypoint_index = new_waypoint_index
        publish_status("Waypoint changed from %d to %d." % (old_waypoint_index, current_waypoint_index))

        if current_waypoint_index == start_waypoint_index:
            publish_status("Detected **START WAYPOINT** (%d). Triggering capture start." % start_waypoint_index)
            start_capture()
        elif current_waypoint_index == stop_waypoint_index:
            publish_status("Detected **STOP WAYPOINT** (%d). Triggering capture stop." % stop_waypoint_index)
            stop_capture()
        else:
            publish_status("Currently at waypoint %d. Waiting for start (%d) or stop (%d) waypoint." % 
                          (current_waypoint_index, start_waypoint_index, stop_waypoint_index))

def on_new_sample(sink):
    global frame_count, first_argus_frame_system_time_ns, \
           first_argus_frame_capture_time_ns, time_offset_ns, \
           timestamp_publisher, csv_writer, is_capturing

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
            publish_status("Synchronization established (new capture session).")
            publish_status("  System time (ns) at first frame: %d" % first_argus_frame_system_time_ns)
            publish_status("  Argus time (ns) of first frame: %d" % first_argus_frame_capture_time_ns)
            publish_status("  Calculated offset (ns): %d" % time_offset_ns)

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
            publish_status("ERROR: Could not map buffer for frame %d." % frame_count)
            buffer.unmap(mapinfo)
            return Gst.FlowReturn.ERROR

        # Check for image data validity (e.g., non-empty)
        if mapinfo.size == 0:
            publish_status("WARNING: Received empty buffer for frame %d. Skipping save." % frame_count)
            buffer.unmap(mapinfo)
            return Gst.FlowReturn.OK
            
        try:
            image_data = np.ndarray(
                (height, width, 4), 
                buffer=mapinfo.data,
                dtype=np.uint8
            )

            bgr_image = cv2.cvtColor(image_data, cv2.COLOR_RGBA2BGR)

            image_filename_local = os.path.join(FRAME_SAVE_DIR, "frame_%06d.jpg" % frame_count) # Padded frame number
            
            # Use try-except for imwrite as well
            if not cv2.imwrite(image_filename_local, bgr_image):
                publish_status("ERROR: Failed to save image file: %s" % image_filename_local)
            else:
                publish_status("Saved frame %d to: %s" % (frame_count, image_filename_local))
        except Exception as e:
            publish_status("ERROR: Image processing/saving failed for frame %d: %s" % (frame_count, e))
        finally:
            buffer.unmap(mapinfo)
        # --- End Image Saving Logic ---

        # --- ROS Publishing Logic (Timestamp only) ---
        ros_time_msg = RosTimeMsg()
        ros_time_msg.data = rospy.Time(0, absolute_timestamp_ns) 
        
        if timestamp_publisher:
            message = "{},{}".format(absolute_timestamp_ns, os.path.basename(image_filename_local))
	    timestamp_publisher.publish(message)
            # No status pub for every frame timestamp, too verbose
            # rospy.loginfo("Published timestamp for frame %d to /camera/timestamps" % frame_count)
        else:
            publish_status("WARNING: Timestamp publisher not initialized! Cannot publish timestamps.")
        # --- End ROS Publishing Logic ---

        # Log to CSV
        try:
            csv_writer.writerow([frame_count, argus_capture_time_ns, absolute_timestamp_ns, human_readable_time_str, os.path.basename(image_filename_local)])
            if csv_file: # Ensure data is written promptly
                csv_file.flush()
        except Exception as e:
            publish_status("ERROR: Writing to CSV failed for frame %d: %s" % (frame_count, e))
            
        # These logs are still useful for detailed local debugging
        # rospy.loginfo("Captured frame %d at:" % frame_count)
        # rospy.loginfo("  Relative Argus (ns): %d" % argus_capture_time_ns)
        # rospy.loginfo("  Absolute (ns):     %d" % absolute_timestamp_ns)
        # rospy.loginfo("  Human Readable Time: %s" % human_readable_time_str)
        # rospy.loginfo("-" * 30)

    return Gst.FlowReturn.OK

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        publish_status("GStreamer: End-Of-Stream reached.")
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        publish_status("GStreamer ERROR: %s from %s. Debug: %s" % (err.message, message.src.get_name(), debug))
        if gst_loop:
            publish_status("FATAL: Critical GStreamer error, shutting down node.")
            gst_loop.quit()
    elif t == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        publish_status("GStreamer WARNING: %s from %s. Debug: %s" % (err.message, message.src.get_name(), debug))
    return True

def cleanup_stuck_camera():
    rospy.loginfo("[CLEANUP] Attempting to clean up any stuck camera processes...")
    try:
        # Find processes using 'nvarguscamerasrc' or related camera libs
        # This is a bit aggressive and might kill other legitimate GStreamer processes
        # if not careful. For dedicated camera nodes, it's often acceptable.
        
        # Option 1: Find process IDs (PIDs) using 'nvarguscamerasrc' in their command line
        # This is generally safer as it targets GStreamer specifically.
        cmd = "pgrep -l -f 'gst-launch-1.0|python.*csi_capture.py|nvarguscamerasrc'"
        output = subprocess.check_output(cmd, shell=True).decode('utf-8')
        
        pids_to_kill = []
        for line in output.splitlines():
            if line.strip():
                pid = int(line.split(' ')[0])
                # Exclude self (the current process)
                if pid != os.getpid():
                    pids_to_kill.append(pid)
        
        if pids_to_kill:
            rospy.logwarn("[CLEANUP] Found potentially stuck camera processes with PIDs: %s" % pids_to_kill)
            for pid in pids_to_kill:
                try:
                    os.kill(pid, signal.SIGTERM) # Try gentle termination first
                    rospy.loginfo("[CLEANUP] Sent SIGTERM to PID %d" % pid)
                except OSError as e:
                    rospy.logwarn("[CLEANUP] Could not kill PID %d: %s" % (pid, e))
            
            # Give some time for processes to shut down
            time.sleep(1) 
            
            # Check if they are still alive and send SIGKILL if necessary
            for pid in pids_to_kill:
                if os.path.exists("/proc/%d" % pid):
                    rospy.logwarn("[CLEANUP] PID %d is still alive after SIGTERM, sending SIGKILL." % pid)
                    try:
                        os.kill(pid, signal.SIGKILL)
                        rospy.loginfo("[CLEANUP] Sent SIGKILL to PID %d" % pid)
                    except OSError as e:
                        rospy.logerr("[CLEANUP] Failed to SIGKILL PID %d: %s" % (pid, e))
        else:
            rospy.loginfo("[CLEANUP] No obvious stuck camera processes found.")

    except subprocess.CalledProcessError:
        rospy.loginfo("[CLEANUP] No matching processes found by pgrep.")
    except Exception as e:
        rospy.logerr("[CLEANUP] An error occurred during camera cleanup: %s" % e)


def gst_thread_main():
    global pipeline, bus, gst_loop, csv_file, csv_writer, is_capturing
    cleanup_stuck_camera() 
    Gst.init(None) # Initialize GStreamer here

    if not os.path.exists(FRAME_SAVE_DIR):
        try:
            os.makedirs(FRAME_SAVE_DIR)
            publish_status("INFO: Created directory: %s" % FRAME_SAVE_DIR)
        except OSError as e:
            publish_status("FATAL: Failed to create frame save directory %s: %s" % (FRAME_SAVE_DIR, e))
            return # Exit thread if directory can't be created

    try:
        csv_file = open(CSV_FILENAME, 'wb') # 'wb' for binary write mode in Python 2
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["FrameNumber", "Timestamp_nanoseconds_relative_camera", "Timestamp_nanoseconds_absolute_epoch", "Datetime_HumanReadable_Local", "Image_Filename"])
        publish_status("INFO: CSV file '%s' opened for writing." % CSV_FILENAME)
    except IOError as e:
        publish_status("FATAL: Failed to open CSV file %s: %s" % (CSV_FILENAME, e))
        return # Exit thread if CSV can't be opened

    try:
        pipeline_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! video/x-raw ! videorate ! video/x-raw,framerate=%d/1 ! nvvidconv ! video/x-raw,format=RGBA ! appsink name=mysink" % int(FPS)
        publish_status("INFO: Attempting to launch GStreamer pipeline: %s" % pipeline_str)
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("mysink")
        if not appsink:
            raise RuntimeError("Could not get appsink element. Check GStreamer pipeline string for 'appsink name=mysink'.")

        appsink.set_property("emit-signals", True)
        appsink.set_property("sync", False)
        appsink.connect("new-sample", on_new_sample)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_message)

        # Decision for initial pipeline state:
        if start_waypoint_index == -1:
            publish_status("INFO: Start waypoint not set (defaulting to -1), starting capture immediately.")
            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to PLAYING state at startup. GStreamer error.")
            is_capturing = True 
            publish_status("GStreamer Pipeline **STARTED** immediately.")
        else:
            ret = pipeline.set_state(Gst.State.NULL) 
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to initial NULL state. GStreamer error.")
            is_capturing = False
            publish_status("INFO: GStreamer Pipeline initialized to NULL state. Waiting for waypoint %d to start." % start_waypoint_index)

        gst_loop = GLib.MainLoop()
        gst_loop.run()

    except Exception as e:
        publish_status("FATAL: GStreamer thread encountered an unhandled error during setup or main loop: %s" % e)
    finally:
        if pipeline:
            publish_status("INFO: Cleaning up GStreamer pipeline.")
            pipeline.set_state(Gst.State.NULL)
        if csv_file:
            csv_file.close()
            publish_status("INFO: CSV file '%s' closed." % CSV_FILENAME)


def main():
    global timestamp_publisher, status_publisher, start_waypoint_index, stop_waypoint_index

    rospy.init_node('camera_capture_node', anonymous=True)
    
    # Initialize Publishers EARLY, before potential errors, so status messages can be sent.
    timestamp_publisher = rospy.Publisher('/camera/timestamps', String, queue_size=5)
    status_publisher = rospy.Publisher('/camera/status', String, queue_size=5) # NEW: Status publisher

    # Give publishers a moment to set up
    rospy.sleep(0.5) 
    publish_status("INFO: Camera Capture ROS Node started and initialized publishers.")

    # Get parameters for start/stop waypoints
    try:
        start_waypoint_index = rospy.get_param('/start_waypoint_index')
        stop_waypoint_index = rospy.get_param('/stop_waypoint_index')
        publish_status("INFO: Successfully loaded parameters: START_WAYPOINT=%d, STOP_WAYPOINT=%d" % (start_waypoint_index, stop_waypoint_index))
    except KeyError as e:
        publish_status("ERROR: Failed to load parameter: %s. Please ensure it's defined in your params.yaml and loaded by the launch file." % e)
        publish_status("INFO: Defaulting start/stop waypoints to -1 (immediate start if not set).")
        start_waypoint_index = -1
        stop_waypoint_index = -1
    
    # Subscribe to MAVROS current waypoint topic
    rospy.Subscriber('/mavros/mission/reached', WaypointReached, waypoint_callback)
    publish_status("INFO: Subscribed to /mavros/mission/reached for waypoint control.")
    publish_status("INFO: Press Ctrl+C to stop this node.")

    # Start the GStreamer pipeline thread.
    gst_thread = threading.Thread(target=gst_thread_main)
    gst_thread.daemon = True 
    gst_thread.start()
    publish_status("INFO: GStreamer initialization thread started.")

    # Keep the ROS node alive
    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        publish_status("INFO: ROS shutdown signal received (Ctrl+C).")
        pass 

    # On ROS shutdown, signal GStreamer thread to quit
    if gst_loop: 
        publish_status("INFO: ROS node shutting down. Signaling GStreamer loop to quit.")
        gst_loop.quit()
    
    gst_thread.join() # Wait for the GStreamer thread to finish
    publish_status("INFO: Camera Capture ROS Node stopped gracefully.")

if __name__ == '__main__':
    main()
