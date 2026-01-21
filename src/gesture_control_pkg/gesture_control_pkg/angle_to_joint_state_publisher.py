#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math

class JointStateMapper(Node):
    def __init__(self):
        super().__init__('angle_to_joint_publisher')
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/detected_angles',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state = JointState()
        self.joint_state.name = [
            'magician_joint_1',           # base (rotation)
            'magician_joint_2',           # shoulder
            'magician_joint_3',           # elbow
            'magician_joint_4',           # wrist
            'magician_joint_prismatic_1'  # slider
        ]
        self.get_logger().info("✅ JointStateMapper ready to receive angles.")

    def callback(self, msg):
        # Expect 3 values: [base, shoulder, elbow]
        base_angle = msg.data[0]   # 0–360 degrees
        shoulder   = msg.data[1]   # rear (up/down)
        elbow      = msg.data[2]   # fore (forward/back)

        # Convert base_angle (deg) → radians, since JointState expects radians
        base_rad = math.radians(base_angle)

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [
            base_rad,   # joint_1: base rotation
            shoulder,   # joint_2
            elbow,      # joint_3
            0.0,        # joint_4 (wrist unused for now)
            0.0         # prismatic joint unused
        ]

        self.publisher.publish(self.joint_state)
        self.get_logger().info(f"📡 Published joint positions: {self.joint_state.position}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
