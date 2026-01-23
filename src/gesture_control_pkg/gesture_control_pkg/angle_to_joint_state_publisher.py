#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


class JointStateMapper(Node):
    def __init__(self):
        super().__init__('angle_to_joint_publisher')

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/detected_angles',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.joint_state = JointState()
        self.joint_state.name = [
            'magician_joint_1',           # base
            'magician_joint_2',           # shoulder
            'magician_joint_3',           # elbow
            'magician_joint_4',           # wrist
            'magician_joint_prismatic_1'  # slider
        ]

        self.get_logger().info("✅ JointStateMapper ready (expects radians).")

    def callback(self, msg):
        """
        Expected msg.data formats:
        - 3 values: [base, shoulder, elbow]
        - 4 values: [base, shoulder, elbow, wrist]
        All values MUST be in radians.
        """

        data = msg.data

        base = data[0] if len(data) > 0 else 0.0
        shoulder = data[1] if len(data) > 1 else 0.0
        elbow = data[2] if len(data) > 2 else 0.0
        wrist = data[3] if len(data) > 3 else 0.0

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [
            base,      # joint_1
            shoulder,  # joint_2
            elbow,     # joint_3
            wrist,     # joint_4
            0.0        # prismatic joint (unused)
        ]

        self.publisher.publish(self.joint_state)

        self.get_logger().debug(
            f"Published joints (rad): {self.joint_state.position}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
