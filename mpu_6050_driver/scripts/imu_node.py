#!/usr/bin/env python3

import time
import smbus
import struct
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf_transformations import quaternion_about_axis
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        global bus, ADDR, IMU_FRAME

        bus = smbus.SMBus(self.declare_parameter('bus', 1).value)
        ADDR = self.declare_parameter('device_address', '0x68').value
        if isinstance(ADDR, str):
            ADDR = int(ADDR, 16)
        
        IMU_FRAME = self.declare_parameter('imu_frame', 'imu_link').value

        bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        self.create_timer(0.02, self.publish_imu)
        self.create_timer(10.0, self.publish_temp)

    def publish_temp(self):
        temp_msg = Temperature()
        temp_msg.header.frame_id = IMU_FRAME
        temp_msg.temperature = read_word_2c(TEMP_H) / 340.0 + 36.53
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        self.temp_pub.publish(temp_msg)

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME

        # Read the acceleration values
        accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
        
        # Calculate a quaternion representing the orientation
        accel = np.array([accel_x, accel_y, accel_z])
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read the gyro values
        gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
        
        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = self.get_clock().now().to_msg()

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()