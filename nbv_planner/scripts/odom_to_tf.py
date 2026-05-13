#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/bluerov2/local_position/odom')
        odom_topic = self.get_parameter('odom_topic').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.origin = None  # set from first message; all poses published relative to it

        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'OdomToTF started. Subscribing to {odom_topic}')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position

        if self.origin is None:
            self.origin = (pos.x, pos.y, pos.z)
            self.get_logger().info(
                f'Origin set to ({self.origin[0]:.3f}, {self.origin[1]:.3f}, {self.origin[2]:.3f})')

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = pos.x - self.origin[0]
        t.transform.translation.y = pos.y - self.origin[1]
        t.transform.translation.z = pos.z - self.origin[2]
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
