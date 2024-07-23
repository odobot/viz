#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

def handle_imu_pose(msg):
    br = TransformBroadcaster(node)
    t = TransformStamped()

    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'plane'
    t.child_frame_id = 'imu_link'
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    br.send_transform(t)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = rclpy.create_node('tf_broadcaster_imu')

    node.create_subscription(Imu, '/imu/data', handle_imu_pose, 10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
