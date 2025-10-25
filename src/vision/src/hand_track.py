#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandTrack(Node):
    def __init__(self):
        super().__init__('hand_track_node')
        self.publisher = self.create_publisher(Int8, 'hand_gesture', 10)
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.2, self.process_frame)
        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.4,
            min_tracking_confidence=0.4
        )

    def is_hand_open(self, lm):
        return lm[8].y < lm[6].y and lm[12].y < lm[10].y and \
               lm[16].y < lm[14].y and lm[20].y < lm[18].y

    def is_fist(self, lm):
        return lm[8].y > lm[6].y and lm[12].y > lm[10].y and \
               lm[16].y > lm[14].y and lm[20].y > lm[18].y

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture = -1 # -1: no gesture, 0: open hand, 1: fist
        if results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            if self.is_hand_open(lm):
                gesture = 1
            elif self.is_fist(lm):
                gesture = 0

        msg = Int8()
        msg.data = gesture
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {gesture}")

        # cv2.imshow("Hand Landmarks", frame)
        # if cv2.waitKey(1) == 27:
        #     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrack()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
