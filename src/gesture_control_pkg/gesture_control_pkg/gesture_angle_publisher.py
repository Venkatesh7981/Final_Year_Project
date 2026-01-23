#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import math
from collections import deque

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


class FingerAnglePublisher(Node):
    def __init__(self):
        super().__init__('gesture_angle_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/detected_angles', 10
        )

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Webcam not accessible.")
            exit(1)

        self.hands = mp_hands.Hands(
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Angle smoothing buffers
        self.history_base = deque(maxlen=10)
        self.history_rear = deque(maxlen=10)
        self.history_forearm = deque(maxlen=10)
        self.history_wrist = deque(maxlen=10)

        # Last known stable angles (degrees)
        self.base = 0.0
        self.rear = 0.0
        self.forearm = 0.0
        self.wrist = 0.0

        self.timer = self.create_timer(0.1, self.detect_and_publish)
        self.get_logger().info("Gesture Angle Publisher Initialized")

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        mode = "IDLE"

        if result.multi_hand_landmarks:
            hand = result.multi_hand_landmarks[0]
            lm = hand.landmark

            # Landmark coordinates
            tips = [8, 12, 16, 20]
            pips = [6, 10, 14, 18]

            fingers_up = 0
            for t, p in zip(tips, pips):
                if lm[t].y < lm[p].y:
                    fingers_up += 1

            index_x = lm[8].x * w
            index_y = lm[8].y * h
            middle_y = lm[12].y * h

            # ---------- GESTURE LOGIC ----------
            if fingers_up == 0:
                mode = "STOP"

            elif fingers_up == 1:
                mode = "BASE"
                raw = self.map_x(index_x, cx)
                self.base = self.smooth(raw, self.history_base)

            elif fingers_up == 2:
                mode = "REAR ARM"
                raw = self.map_y(index_y, cy)
                self.rear = self.smooth(raw, self.history_rear)

            elif fingers_up == 3:
                mode = "FOREARM"
                raw = self.map_y(middle_y, cy)
                self.forearm = self.smooth(raw, self.history_forearm)

            elif fingers_up == 4:
                mode = "WRIST"
                raw = self.map_x(index_x, cx, max_angle=90)
                self.wrist = self.smooth(raw, self.history_wrist)

            elif fingers_up == 5:
                mode = "HOME"
                self.base = 0.0
                self.rear = 0.0
                self.forearm = 0.0
                self.wrist = 0.0

            mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

        # Publish angles (radians)
        msg = Float32MultiArray()
        msg.data = [
            math.radians(self.base),
            math.radians(self.rear),
            math.radians(self.forearm),
            math.radians(self.wrist)
        ]
        self.publisher_.publish(msg)

        # UI
        cv2.putText(frame, f"Mode: {mode}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
        cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

        cv2.imshow("Gesture Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    # ----------------- HELPERS -----------------

    def map_y(self, y, center, max_angle=90):
        delta = center - y
        angle = (delta / center) * max_angle
        return clamp(angle, -max_angle, max_angle)

    def map_x(self, x, center, max_angle=180):
        delta = x - center
        angle = (delta / center) * (max_angle / 2)
        return clamp(angle, -max_angle / 2, max_angle / 2)

    def smooth(self, new, history, alpha=0.3):
        if not history:
            history.append(new)
        else:
            history.append(alpha * new + (1 - alpha) * history[-1])
        return history[-1]

    def on_shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()


def clamp(val, mn, mx):
    return max(mn, min(mx, val))


def main(args=None):
    rclpy.init(args=args)
    node = FingerAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
