# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy, math, threading, time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from serial import Serial

PORT = "/dev/ttyACM0"
BAUDRATE = 9600
IMUNUMBER = 1 # the available devices that need to be connected

SERIAL = Serial(PORT, BAUDRATE, timeout=2)

current_sensor_id = None
sensor_id_list = []

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.thread_ = threading.Thread(target=self.serial_thread)
        self.thread_.start()

    def serial_thread(self):
        global current_sensor_id
        while True:
            serial_line = SERIAL.readline().decode('utf8').strip()
            print(f"{PORT}: {serial_line}")

            if serial_line.startswith("IMU Name:"):
                current_sensor_id = int(serial_line.split(":")[-1].strip())
                print(f"Sensor ID: {current_sensor_id}")
            elif serial_line.startswith("Timestamp (s):"):
                time_value = float(serial_line.split(":")[-1].strip())
                print(f"Time: {time_value}")
            elif serial_line == "IMU Data:":
                # read all imu data: eulerData(X, Y, Z), linearAccel(X, Y, Z)
                data_line = SERIAL.readline().decode('utf8').strip()
                print(f"{data_line}")
                data_line = SERIAL.readline().decode('utf8').strip()
                print(f"{data_line}")
                parsed_data = parse_received_data(data_line)
                if len(parsed_data) == 6:
                    euler_x, euler_y, euler_z, linearAccel_x, linearAccel_y, linearAccel_z = parsed_data
                    euler_x, euler_y, euler_z = -euler_y, euler_z, -euler_x
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu_link"
                    msg.linear_acceleration.x = linearAccel_x
                    msg.linear_acceleration.y = linearAccel_y
                    msg.linear_acceleration.z = linearAccel_z
                    q_x, q_y, q_z, q_w = euler_to_quaternion(euler_x, euler_y, euler_z)
                    msg.orientation.x = q_x
                    msg.orientation.y = q_y
                    msg.orientation.z = q_z
                    msg.orientation.w = q_w
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: %s' % msg)

def main(args=None):
    time.sleep(2.0)

    BLE_IMU_initialization()
    imu_calibration()

    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()

def euler_to_quaternion(roll, pitch, yaw):
    roll = roll * math.pi / 180
    pitch = pitch * math.pi / 180
    yaw = yaw * math.pi / 180

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

def parse_received_data(line):
    """Parse received data from arduino"""
    try:
        euler_x, euler_y, euler_z, linearAccel_x, linearAccel_y, linearAccel_z= [float(value) for value in line.split(",")]
        return euler_x, euler_y, euler_z, linearAccel_x, linearAccel_y, linearAccel_z
    except ValueError:
        pass
    return None

def BLE_IMU_initialization():
    '''Initialization of the Bluetooth and IMU sensors'''

    initial_done = False
    while not initial_done:
        serial_line = SERIAL.readline().decode('utf8').strip()
        print(f"{PORT}: {serial_line}")

        # Set devices number
        if serial_line == "Get IMUs number:":
            SERIAL.write(IMUNUMBER.to_bytes(1, 'big'))
        elif serial_line.startswith("IMU Name:"):
            sensor_id = int(serial_line.split(":")[-1].strip())
            sensor_id_list.append(sensor_id)
            print(f"Sensor ID: {sensor_id}")
        elif serial_line == "All IMUs Initialization Done!":
            print(f"All Sensor IDs listed: {sensor_id_list}")
            initial_done = True

def imu_calibration():
    '''Calibration of the imu sensor'''

    imu_calibrated = False
    while not imu_calibrated:
        serial_line = SERIAL.readline().decode('utf8').strip()
        print(f"{PORT}: {serial_line}")
        if serial_line == "Waiting calibration!":
            start_calib = 1
            SERIAL.write(start_calib.to_bytes(1, 'big'))
        elif serial_line == "All IMU sensors calibration done!":
            imu_calibrated = True

if __name__ == '__main__':
    main()
