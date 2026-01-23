#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ManualAnglePublisher(Node):
    def __init__(self):
        super().__init__('manual_angle_publisher')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/detected_angles',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_angles)

        self.get_logger().info(
            "ManualAnglePublisher started (publishing full joint angles)."
        )

    def publish_angles(self):
        """
        Joint order (radians):
        [base, shoulder, elbow, wrist]
        """

        msg = Float32MultiArray()

        msg.data = [
            0.0,     # base rotation
            -0.4,    # shoulder (rear arm)
            0.6,     # elbow (forearm)
            0.0      # wrist
        ]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Manual angles published (rad): {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManualAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
