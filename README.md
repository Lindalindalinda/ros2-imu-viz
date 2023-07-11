# ros2-imu-viz

An IMU data publisher for ROS2, written in Python, by Jiajing Li.

This code reads the IMU data from the serial port and converts the Euler angles to quaternions, then posts to the `/imu/data` topic.

## Setup

This code is developed and tested on Ubuntu 20.04. To install ROS2 on Ubuntu, check out [https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html).

**1. Clone**

```bash
git clone https://github.com/Lindalindalinda/ros2-imu-viz.git
```

**2. Config**

Open `ros2-imu-viz/src/py_pubsub/py_pubsub/publisher_member_function.py`, change line 20 to the serial port to which the IMU is connected. This value is `"/dev/ttyACM0"` by default. If your IMU is connected to COM3 (on Windows), then change it to:

```python
PORT = "COM3"
```

**3. Build**

Note: change the second line `source install/setup.bash` to `source install/setup.zsh` if you are using Zsh instead of Bash.

```bash
cd ros2-imu-viz
source /opt/ros/foxy/setup.bash
colcon build --packages-select py_pubsub
source install/setup.bash
```

**4. Run**

```bash
ros2 run py_pubsub talker
```

After the publisher connects to IMU, it will continuously post messages in the topic `/imu/data`. A message has the following parameters:

+ Linear acceleration:
  + `linear_acceleration.x`
  + `linear_acceleration.y`
  + `linear_acceleration.z`
+ Orientation
  + `orientation.x`
  + `orientation.y`
  + `orientation.z`
  + `orientation.w`

## Subscriber Example

There is a example subscriber in `pybullet-imu-viz` directory, which is modified from the subscriber provided by [https://robofoundry.medium.com/simple-pybullet-ros2-node-to-visualize-sensor-data-c88d97f18d7d](https://robofoundry.medium.com/simple-pybullet-ros2-node-to-visualize-sensor-data-c88d97f18d7d).

**1. Build**

```bash
cd pybullet-imu-viz
colcon build
source install/setup.bash
```

**2. Run**

To start the subscriber and the simulation (PyBullet), execute:

```bash
ros2 launch imu_listener imu_pybullet.launch.py
```

