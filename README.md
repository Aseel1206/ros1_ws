# ROS1 WS

# Installation
System Requirements:
Ubuntu 18 (This gudie uses jetson nano. Based on ubuntu version ros distribution will vary.)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update
sudo apt install ros-melodic-desktop
```
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

# Cloning Package and setup

```
git clone https://github.com/Aseel1206/ros1_ws.git
```
```
sudo apt install python-catkin-tools
```
```
cd ros1_ws
catkin build
```

# Mavros Installation

```
sudo apt-get install ros-noetic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

# Launching Nodes

Launching Individual Nodes:
```
rosrun drone_ros demo_node.py
```

Launching Mavros:
UDP:
```
roslaunch mavros apm.launch fcu_url:="udp://192.168.1.48:14450@14555"
```
USB:
```
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

# Launching Nodes from launch file
```
roslaunch drone_ros start_all.launch
```

# Code Documentation

##  Camera Capture Node

This ROS node captures frames from a GStreamer-based camera pipeline, saves them as images, logs precise timestamps, and publishes status and timestamp messages.

---

###  Features

- Start and stop image capture based on MAVROS mission waypoints.
- Save frames as `.jpg` images.
- Publish accurate absolute timestamps to a ROS topic.
- Log all frame metadata to a CSV file.
- Publish runtime status updates to a ROS topic.

---

### 🛠️ Dependencies

#### Python Packages

- `rospy`
- `cv2` (OpenCV)
- `numpy`
- `gi` (GObject Introspection)

#### ROS Packages

- `mavros`
- `mavros_msgs`
- `std_msgs`
- `sensor_msgs` (optional)
- `cv_bridge` (optional if image topics are used)

---

### Published Topics

- /camera/timestamps	
- /camera/status

### Subscribed Topics

- /mavros/mission/reached

### Params

```
<param name="start_waypoint" value="2"/>
<param name="stop_waypoint" value="5"/>
```
---

## Geotagger Node

This ROS 1 node synchronizes GPS, IMU, and camera image data to perform geotagging — embedding geographic and orientation metadata into a CSV file for each camera frame.

### 🛰️ Features

- Subscribes to:
  - `/camera/timestamps` (image timestamp and filename as `String`)
  - `/mavros/imu/data` (IMU data)
  - `/mavros/global_position/global` (GPS coordinates)
- Interpolates GPS and IMU data to match image timestamps
- Extracts orientation (yaw) from quaternion IMU data
- Saves geotagged data (timestamp, GPS, orientation, yaw, filename) to a CSV
- Publishes status updates to `/geotagger/status`

---

### 📂 Output

Geotagged data will be saved to:
- /home/edhitha/geotag_output/geotag_data.csv

#### 
Each row of the CSV includes:
- [timestamp, latitude, longitude, altitude, orientation_w, yaw, filename]

  
---

### 🧰 Dependencies

Ensure the following packages are installed:

- ROS 1 (tested with Noetic)
- `rospy`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `numpy`
- `opencv-python` (optional for future image handling)
- `mavros` (for GPS and IMU topics)

---

## Drone Status Monitor

`drone_status_monitor` is a ROS node designed to monitor the status of various components of a drone system, including MAVROS connection, GPS, IMU, camera, geotagger, and timestamps. It publishes a JSON-formatted summary to `/drone_status` and prints a detailed log to the terminal every second.

### Features

- Monitors ROS time and FCU (Flight Control Unit) time.
- Tracks:
  - MAVROS connection status
  - IMU data availability
  - GPS fix availability
  - Geotagger node and status
  - Camera node and operational status
- Publishes a JSON status string on the `/drone_status` topic.
- Prints detailed terminal output for debugging or live feedback.

### Dependencies

- ROS (Tested on ROS Noetic, but should work with other ROS 1 distributions)
- MAVROS
- Python packages:
  - `rospy`
  - `rosnode`
  - `std_msgs`
  - `sensor_msgs`
  - `mavros_msgs`

---

# Troubleshooting

## illegal Core Exception
- check if ```pyton``` returns python2 in terminal. If not set python to python2.

## Permission Denied when using usb connection for fc
```
sudo usermod -a -G dialout $USER
```

## No gps and imu data from fc though usb
- In Mission planner, Set usb, baud rate, gps data frequency, etc

