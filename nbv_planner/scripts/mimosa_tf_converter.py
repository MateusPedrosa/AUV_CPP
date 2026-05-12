#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class MimosaTFConverter(Node):
    def __init__(self):
        super().__init__('mimosa_tf_converter')

        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)

        self.subscription = self.create_subscription(
            TransformStamped,
            '/mimosa_node/graph/transform',
            self.transform_callback,
            10
        )
        self.get_logger().info('TF Converter Node started. Listening to /mimosa_node/graph/transform...')

    def transform_callback(self, msg: TransformStamped):
        msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg = TFMessage()
        tf_msg.transforms.append(msg)
        self.tf_publisher.publish(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MimosaTFConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
