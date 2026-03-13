#!/usr/bin/env python3
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class LidColorClassifier(Node):
    """
    Subscribes:  /camera/color/image_raw  (sensor_msgs/Image)
    Publishes:   /product_class           (std_msgs/String) -> "red" | "blue" | "unknown"
                 /product_class_debug     (std_msgs/String) -> text w/ scores
    """

    def __init__(self):
        super().__init__("vision_lid_classifier")

        # ---- Params ----
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("publish_debug", True)

        # Thresholds / ROI
        self.declare_parameter("min_area_ratio", 0.005)  # área mínima (fração da imagem) para aceitar cor
        self.declare_parameter("roi_ymin", 0.20)          # recorta parte superior/inferior (0..1)
        self.declare_parameter("roi_ymax", 0.90)
        self.declare_parameter("roi_xmin", 0.10)
        self.declare_parameter("roi_xmax", 0.90)

        # HSV thresholds (ajustáveis)
        # Vermelho costuma “dar a volta” no hue: (0..10) e (160..179)
        self.declare_parameter("red_h1_low", 0)
        self.declare_parameter("red_h1_high", 10)
        self.declare_parameter("red_h2_low", 160)
        self.declare_parameter("red_h2_high", 179)
        self.declare_parameter("red_s_low", 80)
        self.declare_parameter("red_v_low", 50)

        # Azul
        self.declare_parameter("blue_h_low", 95)
        self.declare_parameter("blue_h_high", 135)
        self.declare_parameter("blue_s_low", 80)
        self.declare_parameter("blue_v_low", 50)

        self.bridge = CvBridge()

        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.pub_class = self.create_publisher(String, "/product_class", 10)
        self.pub_dbg = self.create_publisher(String, "/product_class_debug", 10)

        self.get_logger().info(f"Vision node OK. Listening on: {image_topic}")

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w = bgr.shape[:2]

        # ROI crop (ajuda a ignorar fundo)
        roi = self._crop_roi(bgr)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_area = self._mask_area_red(hsv)
        blue_area = self._mask_area_blue(hsv)

        roi_area = roi.shape[0] * roi.shape[1]
        red_ratio = red_area / max(1, roi_area)
        blue_ratio = blue_area / max(1, roi_area)

        min_area_ratio = float(self.get_parameter("min_area_ratio").value)

        label = "unknown"
        if red_ratio > min_area_ratio or blue_ratio > min_area_ratio:
            label = "red" if red_ratio > blue_ratio else "blue"

        self.pub_class.publish(String(data=label))

        if bool(self.get_parameter("publish_debug").value):
            dbg = f"red_ratio={red_ratio:.4f} blue_ratio={blue_ratio:.4f} min={min_area_ratio:.4f} -> {label}"
            self.pub_dbg.publish(String(data=dbg))

    def _crop_roi(self, img):
        h, w = img.shape[:2]
        y0 = int(float(self.get_parameter("roi_ymin").value) * h)
        y1 = int(float(self.get_parameter("roi_ymax").value) * h)
        x0 = int(float(self.get_parameter("roi_xmin").value) * w)
        x1 = int(float(self.get_parameter("roi_xmax").value) * w)
        y0, y1 = max(0, y0), min(h, y1)
        x0, x1 = max(0, x0), min(w, x1)
        if y1 <= y0 or x1 <= x0:
            return img
        return img[y0:y1, x0:x1]

    def _mask_area_red(self, hsv):
        h1l = int(self.get_parameter("red_h1_low").value)
        h1h = int(self.get_parameter("red_h1_high").value)
        h2l = int(self.get_parameter("red_h2_low").value)
        h2h = int(self.get_parameter("red_h2_high").value)
        sl = int(self.get_parameter("red_s_low").value)
        vl = int(self.get_parameter("red_v_low").value)

        lower1 = np.array([h1l, sl, vl], dtype=np.uint8)
        upper1 = np.array([h1h, 255, 255], dtype=np.uint8)
        lower2 = np.array([h2l, sl, vl], dtype=np.uint8)
        upper2 = np.array([h2h, 255, 255], dtype=np.uint8)

        m1 = cv2.inRange(hsv, lower1, upper1)
        m2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(m1, m2)

        mask = self._cleanup(mask)
        return int(cv2.countNonZero(mask))

    def _mask_area_blue(self, hsv):
        hl = int(self.get_parameter("blue_h_low").value)
        hh = int(self.get_parameter("blue_h_high").value)
        sl = int(self.get_parameter("blue_s_low").value)
        vl = int(self.get_parameter("blue_v_low").value)

        lower = np.array([hl, sl, vl], dtype=np.uint8)
        upper = np.array([hh, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        mask = self._cleanup(mask)
        return int(cv2.countNonZero(mask))

    @staticmethod
    def _cleanup(mask):
        # remove ruído
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask


def main():
    rclpy.init()
    node = LidColorClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
