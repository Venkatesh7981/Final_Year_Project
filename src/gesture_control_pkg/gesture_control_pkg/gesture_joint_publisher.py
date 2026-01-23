#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


class ArmAnglePublisher(Node):
    def __init__(self):
        super().__init__('gesture_angle_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/detected_angles', 10
        )

        self.pose = mp_pose.Pose(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Webcam not accessible.")
            exit(1)

        self.last_rear = 0.0
        self.last_forearm = 0.0

        self.timer = self.create_timer(0.1, self.detect_and_publish)
        self.get_logger().info("Pose-Based Gesture Angle Publisher Initialized")

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)

        mode = "IDLE"

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            h, w, _ = frame.shape

            shoulder = np.array([
                lm[mp_pose.PoseLandmark.LEFT_SHOULDER].x * w,
                lm[mp_pose.PoseLandmark.LEFT_SHOULDER].y * h
            ])
            elbow = np.array([
                lm[mp_pose.PoseLandmark.LEFT_ELBOW].x * w,
                lm[mp_pose.PoseLandmark.LEFT_ELBOW].y * h
            ])
            wrist = np.array([
                lm[mp_pose.PoseLandmark.LEFT_WRIST].x * w,
                lm[mp_pose.PoseLandmark.LEFT_WRIST].y * h
            ])

            elbow_angle = calculate_angle(shoulder, elbow, wrist)

            # ---------- GESTURE LOGIC ----------
            if wrist[1] > shoulder[1]:
                mode = "STOP"

            elif elbow_angle > 160:
                mode = "HOME"
                self.last_rear = 0.0
                self.last_forearm = 0.0

            else:
                mode = "ACTIVE"
                rear = elbow_angle - 90
                forearm = calculate_angle(elbow, wrist, shoulder) - 90

                self.last_rear = clamp(rear, -5, 80)
                self.last_forearm = clamp(forearm, -10, 85)

            # Draw skeleton
            mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS
            )

        # Publish angles (always safe)
        msg = Float32MultiArray()
        msg.data = [
            math.radians(self.last_rear),
            math.radians(self.last_forearm)
        ]
        self.publisher_.publish(msg)

        # UI
        cv2.putText(frame, f"Mode: {mode}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Rear: {self.last_rear:.1f}°", (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Forearm: {self.last_forearm:.1f}°", (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Pose Gesture Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def on_shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()


def calculate_angle(a, b, c):
    ba = a - b
    bc = c - b
    cosine = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine, -1.0, 1.0))
    return np.degrees(angle)


def clamp(val, mn, mx):
    return max(mn, min(mx, val))


def main(args=None):
    rclpy.init(args=args)
    node = ArmAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
