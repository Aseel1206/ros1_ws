#!/usr/bin/env python
# Compatible with ROS1 Python 2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import os
import time
import datetime
import threading
import json
import yaml
import signal

class JetsonGstreamerROSNode:
    def __init__(self):
        """Initialize ROS node with GStreamer USB camera capture"""
        # Initialize ROS node
        rospy.init_node('jetson_camera_capture', anonymous=True)
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Get ROS parameters with defaults
        self.device_id = rospy.get_param('~device_id', 0)
        self.target_fps = rospy.get_param('~fps', 1.0)
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.output_dir = rospy.get_param('~output_dir', 'captured_frames')
        self.log_file_opened = False

        # Create publisher for image metadata
        self.image_pub = rospy.Publisher('~image_metadata', String, queue_size=10)
	self.stop_sub = rospy.Subscriber('~stop_capture', String, self.stop_capture_callback)
        self.stop_requested = False

        # Setup signal handler to catch Ctrl+C cleanly
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Internal variables
        self.pipeline = None
        self.sink = None
        self.bus = None
        self.mainloop = None
        self.running = False
        self.frame_count = 0
        self.last_capture_time = 0
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Log file for timestamps
        self.log_file = os.path.join(self.output_dir, "capture_timestamps.csv")
        with open(self.log_file, 'w') as f:
            f.write("frame_number,filename,timestamp_unix,timestamp_iso,gst_timestamp,ros_time\n")
        
        rospy.loginfo("Jetson Camera Capture Node initialized with parameters:")
        rospy.loginfo("  Device ID: %s" % self.device_id)
        rospy.loginfo("  Target FPS: %s" % self.target_fps)
        rospy.loginfo("  Resolution: %sx%s" % (self.width, self.height))
        rospy.loginfo("  Output Directory: %s" % self.output_dir)

    def stop_capture_callback(self, msg):
        """ROS subscriber callback to stop capture on demand"""
        if msg.data.lower() == "true" or msg.data == "1":
            rospy.loginfo("Stop capture requested via topic")
            self.stop_requested = True
            self.stop()

    def signal_handler(self, signum, frame):
        """Handle SIGINT and SIGTERM signals for graceful shutdown"""
        rospy.loginfo("Signal %d received, stopping capture..." % signum)
        self.stop()
        rospy.signal_shutdown("Signal received")

    def build_pipeline(self):
        """Build the GStreamer pipeline for USB camera capture with NVIDIA acceleration"""
        # For USB cameras with hardware acceleration on Jetson
        # This is for CSI cameras, not typical USB cameras unless specifically configured.
	# If your camera is truly a USB webcam, this will not work.
	pipeline_str = (
	    "nvarguscamerasrc sensor-id=%s ! "
	    "video/x-raw(memory:NVMM),width=%s,height=%s,framerate=%s/1,format=NV12 ! "
	    "nvvidconv flip-method=0 ! " # Add flip-method if needed
	    "video/x-raw,format=BGRx ! "
	    "videoconvert ! "
	    "video/x-raw,format=BGR ! "
	    "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
	) % (self.device_id, self.width, self.height, int(self.target_fps))
		
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            rospy.loginfo("Using hardware-accelerated pipeline")
        except GLib.Error as e:
            rospy.logwarn("Error creating hardware-accelerated pipeline: %s" % e)
            fallback_pipeline_str = (
                "v4l2src device=/dev/video%s ! "
                "video/x-raw,width=%s,height=%s ! "
                "videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
            ) % (self.device_id, self.width, self.height)
            rospy.loginfo("Trying fallback pipeline without hardware acceleration")
            self.pipeline = Gst.parse_launch(fallback_pipeline_str)
        
        # Get the appsink element
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self.on_new_sample)
        
        # Set up bus for pipeline messages
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_bus_message)

    def on_bus_message(self, bus, message):
        t = message.type

        if t == Gst.MessageType.EOS:
            rospy.loginfo("End of stream")
            if not rospy.is_shutdown():
                self.stop()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            rospy.logerr("Error: %s, %s" % (err, debug))
            if not rospy.is_shutdown():
                self.stop()

        return True

    def on_new_sample(self, sink):
        """Process a new frame from the GStreamer pipeline"""
        now = time.time()
        ros_time = rospy.Time.now()
        
        # Check if we should capture this frame based on our target FPS
        if self.last_capture_time > 0 and (now - self.last_capture_time) < (1.0 / self.target_fps):
            # Skip this frame to maintain FPS
            return Gst.FlowReturn.OK
        
        # Get the sample from the sink
        sample = sink.emit("pull-sample")
        if sample:
            # Get the buffer from the sample
            buffer = sample.get_buffer()
            
            # Get the timestamp from the buffer
            gst_timestamp = buffer.pts / Gst.SECOND
            
            # Get buffer data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                rospy.logerr("Failed to map buffer")
                return Gst.FlowReturn.ERROR
            
            # Convert buffer to numpy array
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            
            # Create numpy array from buffer data
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )
            
            # Get timestamp immediately after capture
            timestamp = time.time()
            timestamp_iso = datetime.datetime.fromtimestamp(timestamp).isoformat()
            
            # Process and save the frame
            self.process_frame(frame, timestamp, timestamp_iso, gst_timestamp, ros_time)
            
            # Unmap the buffer
            buffer.unmap(map_info)
            
            # Update last capture time
            self.last_capture_time = timestamp
            
            return Gst.FlowReturn.OK
        
        return Gst.FlowReturn.ERROR

    def process_frame(self, frame, timestamp, timestamp_iso, gst_timestamp, ros_time):
	
        """Process and save a captured frame with timestamps"""
        # Format filename with frame number and timestamp
        filename = "frame_%06d_%d.jpg" % (self.frame_count, int(timestamp*1000))
        filepath = os.path.join(self.output_dir, filename)
        
        # Save the frame
        cv2.imwrite(filepath, frame)
        
        # Log the timestamp in file
        with open(self.log_file, 'a') as f:
            f.write("%d,%s,%s,%s,%s,%s\n" % (
                self.frame_count, filename, timestamp, timestamp_iso, 
                gst_timestamp, ros_time.to_sec()))
    
        # Create metadata dictionary
        metadata = {
            "frame_number": self.frame_count,
            "filename": filename,
            "filepath": filepath,
            "timestamp_unix": timestamp,
            "timestamp_iso": timestamp_iso,
            "gst_timestamp": gst_timestamp,
            "ros_time_secs": ros_time.secs,
            "ros_time_nsecs": ros_time.nsecs,
            "width": frame.shape[1],
            "height": frame.shape[0]
        }
        
        # Publish metadata as JSON string
        metadata_json = json.dumps(metadata)
        self.image_pub.publish(metadata_json)
        
        rospy.loginfo("Frame %d: Captured at %s and published to topic" % (
            self.frame_count, timestamp_iso))
        self.frame_count += 1

    def start(self):
        """Start the capture pipeline"""
        self.build_pipeline()
        
        # Start playing
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            rospy.logerr("Failed to start pipeline")
            return False
        
        # Start the main loop in a separate thread
        self.mainloop = GLib.MainLoop()
        self.running = True
        
        # Create and start thread
        self.thread = threading.Thread(target=self.mainloop.run)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo("Started capture at %s FPS." % self.target_fps)
        self.last_capture_time = time.time()
        return True

    def stop(self):
        """Stop the capture pipeline"""
        if self.running:
            self.running = False
            if self.mainloop:
                GLib.idle_add(self.mainloop.quit)
                self.mainloop = None
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
            rospy.loginfo("Capture ended. %d frames saved to %s" % (
                self.frame_count, self.output_dir))
            if self.log_file_opened:
                rospy.loginfo("Timestamp log saved to %s" % self.log_file)
            else:
                # If no frames captured, remove empty CSV log file
                if os.path.exists(self.log_file):
                    os.remove(self.log_file)
            self.log_file_opened = False

    def run(self):
        if self.start():
            while not rospy.is_shutdown() and not self.stop_requested:
                rospy.sleep(0.1)  # short sleep loop instead of spin()
            self.stop()
        else:
            rospy.logerr("Failed to start camera capture")


def main():
    try:
        node = JetsonGstreamerROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
