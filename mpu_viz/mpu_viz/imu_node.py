#!/usr/bin/env python3

import time
import struct
import sys
import numpy as np
import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, Imu
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
import sys
mpu_6050_driver = sys.path.append('/home/slambot/Desktop/ros_ws/src/mpu_6050_driver')
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = Node.get_clock().now().to_msg()
    temp_pub.publish(temp_msg)

def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration values
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

    imu_msg.linear_acceleration.x = accel_x * 9.8
    imu_msg.linear_acceleration.y = accel_y * 9.8
    imu_msg.linear_acceleration.z = accel_z * 9.8

    # Read the gyro values
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

    imu_msg.angular_velocity.x = gyro_x * 0.0174
    imu_msg.angular_velocity.y = gyro_y * 0.0174
    imu_msg.angular_velocity.z = gyro_z * 0.0174

    imu_msg.header.stamp = Node.get_clock().now().to_msg()

    imu_pub.publish(imu_msg)

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        
        global bus, ADDR, IMU_FRAME

        bus = smbus.SMBus(self.get_parameter('bus').value)
        ADDR = self.get_parameter('device_address').value
        if isinstance(ADDR, str):
            ADDR = int(ADDR, 16)

        IMU_FRAME = self.get_parameter('imu_frame').value

        bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.imu_timer = self.create_timer(0.02, publish_imu)
        self.temp_timer = self.create_timer(10.0, publish_temp)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
