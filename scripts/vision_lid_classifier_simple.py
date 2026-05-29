#!/usr/bin/env python3
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class LidColorClassifierSimple(Node):
    """
    Versão simples:
    - usa ROI central da imagem
    - ignora a faixa inferior (texto ETS)
    - segmenta vermelho/azul em HSV
    - escolhe o melhor blob por score
    """

    def __init__(self):
        super().__init__("vision_lid_classifier_simple")

        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("show_debug_windows", True)

        # ROI central
        self.declare_parameter("roi_xmin", 0.20)
        self.declare_parameter("roi_xmax", 0.80)
        self.declare_parameter("roi_ymin", 0.15)
        self.declare_parameter("roi_ymax", 0.85)

        # cortar parte inferior da ROI onde fica texto
        self.declare_parameter("bottom_cut_ratio", 0.22)

        # área mínima/máxima do blob relativo à ROI útil
        self.declare_parameter("min_blob_area_ratio", 0.01)
        self.declare_parameter("max_blob_area_ratio", 0.45)

        # HSV vermelho
        self.declare_parameter("red_h1_low", 0)
        self.declare_parameter("red_h1_high", 8)
        self.declare_parameter("red_h2_low", 170)
        self.declare_parameter("red_h2_high", 179)
        self.declare_parameter("red_s_low", 90)
        self.declare_parameter("red_v_low", 60)

        # HSV azul/ciano
        self.declare_parameter("blue_h_low", 88)
        self.declare_parameter("blue_h_high", 130)
        self.declare_parameter("blue_s_low", 80)
        self.declare_parameter("blue_v_low", 60)

        self.declare_parameter("morph_kernel", 5)

        self.bridge = CvBridge()

        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.pub_class = self.create_publisher(String, "/product_class", 10)
        self.pub_dbg = self.create_publisher(String, "/product_class_debug", 10)

        self.get_logger().info(f"Simple vision node listening on: {image_topic}")

    def on_image(self, msg: Image):
        if not hasattr(self, '_img_count'):
            self._img_count = 0
        self._img_count += 1
        if self._img_count % 30 == 0:
            self.get_logger().info(f"Heartbeat: Received {self._img_count} images from camera...")
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_image(bgr, is_webcam_test=False)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    def process_image(self, bgr, is_webcam_test=False):
        debug_vis = bgr.copy()

        roi, roi_box = self._crop_roi(bgr)
        if roi is None:
            label = "unknown"
            dbg_text = "invalid_roi"
            self._publish_result(label, dbg_text, is_webcam_test)
            self._draw_status(debug_vis, label, dbg_text)
            self._show_windows(debug_vis, None, None, None)
            return

        x0, y0, x1, y1 = roi_box
        roi_h, roi_w = roi.shape[:2]

        # máscara útil da ROI: corta parte inferior
        valid_mask = np.ones((roi_h, roi_w), dtype=np.uint8) * 255
        y_cut = int(roi_h * (1.0 - float(self.get_parameter("bottom_cut_ratio").value)))
        cv2.rectangle(valid_mask, (0, y_cut), (roi_w, roi_h), 0, -1)

        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        red_mask = self._mask_red(hsv)
        blue_mask = self._mask_blue(hsv)

        red_mask = cv2.bitwise_and(red_mask, valid_mask)
        blue_mask = cv2.bitwise_and(blue_mask, valid_mask)

        red_mask = self._cleanup(red_mask)
        blue_mask = self._cleanup(blue_mask)

        red_info = self._best_blob_info(red_mask, roi.shape[:2], "red")
        blue_info = self._best_blob_info(blue_mask, roi.shape[:2], "blue")

        label, dbg_text, chosen_info = self._decide_label(red_info, blue_info)
        self._publish_result(label, dbg_text, is_webcam_test)

        # debug
        cv2.rectangle(debug_vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.line(debug_vis, (x0, y0 + y_cut), (x1, y0 + y_cut), (100, 255, 100), 2)

        if red_info is not None:
            approx = red_info["approx"] + np.array([[[x0, y0]]], dtype=np.int32)
            cv2.drawContours(debug_vis, [approx], -1, (0, 0, 255), 3)
            c = red_info["center"]
            c_global = (c[0] + x0, c[1] + y0)
            cv2.circle(debug_vis, c_global, 4, (0, 0, 255), -1)

        if blue_info is not None:
            approx = blue_info["approx"] + np.array([[[x0, y0]]], dtype=np.int32)
            cv2.drawContours(debug_vis, [approx], -1, (255, 0, 0), 3)
            c = blue_info["center"]
            c_global = (c[0] + x0, c[1] + y0)
            cv2.circle(debug_vis, c_global, 4, (255, 0, 0), -1)

        if chosen_info is not None:
            approx = chosen_info["approx"] + np.array([[[x0, y0]]], dtype=np.int32)
            cv2.drawContours(debug_vis, [approx], -1, (0, 255, 255), 4)

        self._draw_status(debug_vis, label, dbg_text)
        self._show_windows(debug_vis, red_mask, blue_mask, valid_mask)

    def _crop_roi(self, img):
        h, w = img.shape[:2]

        x0 = int(w * float(self.get_parameter("roi_xmin").value))
        x1 = int(w * float(self.get_parameter("roi_xmax").value))
        y0 = int(h * float(self.get_parameter("roi_ymin").value))
        y1 = int(h * float(self.get_parameter("roi_ymax").value))

        x0 = max(0, min(w - 1, x0))
        x1 = max(1, min(w, x1))
        y0 = max(0, min(h - 1, y0))
        y1 = max(1, min(h, y1))

        if x1 <= x0 or y1 <= y0:
            return None, None

        return img[y0:y1, x0:x1].copy(), (x0, y0, x1, y1)

    def _mask_red(self, hsv):
        h1l = int(self.get_parameter("red_h1_low").value)
        h1h = int(self.get_parameter("red_h1_high").value)
        h2l = int(self.get_parameter("red_h2_low").value)
        h2h = int(self.get_parameter("red_h2_high").value)
        sl = int(self.get_parameter("red_s_low").value)
        vl = int(self.get_parameter("red_v_low").value)

        m1 = cv2.inRange(
            hsv,
            np.array([h1l, sl, vl], dtype=np.uint8),
            np.array([h1h, 255, 255], dtype=np.uint8),
        )
        m2 = cv2.inRange(
            hsv,
            np.array([h2l, sl, vl], dtype=np.uint8),
            np.array([h2h, 255, 255], dtype=np.uint8),
        )
        return cv2.bitwise_or(m1, m2)

    def _mask_blue(self, hsv):
        hl = int(self.get_parameter("blue_h_low").value)
        hh = int(self.get_parameter("blue_h_high").value)
        sl = int(self.get_parameter("blue_s_low").value)
        vl = int(self.get_parameter("blue_v_low").value)

        return cv2.inRange(
            hsv,
            np.array([hl, sl, vl], dtype=np.uint8),
            np.array([hh, 255, 255], dtype=np.uint8),
        )

    def _best_blob_info(self, mask, roi_shape, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        h, w = roi_shape
        usable_h = int(h * (1.0 - float(self.get_parameter("bottom_cut_ratio").value)))
        usable_area = max(1, w * usable_h)

        min_ratio = float(self.get_parameter("min_blob_area_ratio").value)
        max_ratio = float(self.get_parameter("max_blob_area_ratio").value)

        cx_ref = w / 2.0
        cy_ref = usable_h / 2.0

        best = None
        best_score = -1e9

        for cnt in contours:
            area = cv2.contourArea(cnt)
            area_ratio = area / usable_area

            if area_ratio < min_ratio or area_ratio > max_ratio:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw < 10 or bh < 10:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]

            dist = np.hypot(cx - cx_ref, cy - cy_ref) / max(w, h)
            if dist > 0.35:
                continue

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / max(hull_area, 1.0)

            perimeter = cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, 0.04 * perimeter, True)
            vertices = len(approx)

            rect = cv2.minAreaRect(cnt)
            rw, rh = rect[1]
            if rw < 1 or rh < 1:
                continue
            aspect = max(rw, rh) / max(min(rw, rh), 1.0)

            extent = area / max(bw * bh, 1.0)

            score = 0.0
            score += 2.2 * area_ratio
            score += 1.6 * (1.0 - min(dist * 2.0, 1.0))
            score += 1.0 * min(solidity, 1.0)

            if color_name == "red":
                if 3 <= vertices <= 5:
                    score += 2.0
                if 0.30 <= extent <= 0.75:
                    score += 1.5
                if 0.7 <= aspect <= 1.8:
                    score += 1.0

            elif color_name == "blue":
                if 4 <= vertices <= 6:
                    score += 2.0
                if 0.60 <= extent <= 1.0:
                    score += 1.5
                if 0.7 <= aspect <= 1.5:
                    score += 1.0

            if score > best_score:
                best_score = score
                best = {
                    "contour": cnt,
                    "approx": approx,
                    "score": float(score),
                    "area_ratio": float(area_ratio),
                    "vertices": int(vertices),
                    "extent": float(extent),
                    "aspect": float(aspect),
                    "solidity": float(solidity),
                    "center": (int(cx), int(cy)),
                }

        return best

    def _decide_label(self, red_info, blue_info):
        parts = []

        if red_info is not None:
            parts.append(f"R={red_info['score']:.2f},v={red_info['vertices']},e={red_info['extent']:.2f}")
        else:
            parts.append("R=0")

        if blue_info is not None:
            parts.append(f"B={blue_info['score']:.2f},v={blue_info['vertices']},e={blue_info['extent']:.2f}")
        else:
            parts.append("B=0")

        if red_info is None and blue_info is None:
            return "unknown", " ".join(parts), None

        if red_info is not None and blue_info is None:
            return "red", " ".join(parts), red_info

        if blue_info is not None and red_info is None:
            return "blue", " ".join(parts), blue_info

        margin = 0.8
        if red_info["score"] > blue_info["score"] + margin:
            return "red", " ".join(parts), red_info
        if blue_info["score"] > red_info["score"] + margin:
            return "blue", " ".join(parts), blue_info

        if red_info["area_ratio"] > blue_info["area_ratio"]:
            return "red", " ".join(parts) + " tie=area", red_info
        if blue_info["area_ratio"] > red_info["area_ratio"]:
            return "blue", " ".join(parts) + " tie=area", blue_info

        return "unknown", " ".join(parts) + " ambiguous", None

    def _cleanup(self, mask):
        ksize = int(self.get_parameter("morph_kernel").value)
        if ksize < 3:
            ksize = 3
        if ksize % 2 == 0:
            ksize += 1

        k = np.ones((ksize, ksize), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        return mask

    def _publish_result(self, label, dbg_text, is_webcam_test):
        if is_webcam_test:
            return
        self.pub_class.publish(String(data=label))
        if bool(self.get_parameter("publish_debug").value):
            self.pub_dbg.publish(String(data=f"{label} | {dbg_text}"))

    def _draw_status(self, img, label, dbg_text):
        if label == "red":
            color = (0, 0, 255)
            text = "RED TRIANGLE"
        elif label == "blue":
            color = (255, 0, 0)
            text = "BLUE SQUARE"
        else:
            color = (160, 160, 160)
            text = "UNKNOWN"

        cv2.putText(img, f"Class: {text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(img, dbg_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    def _show_windows(self, debug_vis, red_mask, blue_mask, valid_mask):
        if not bool(self.get_parameter("show_debug_windows").value):
            return

        cv2.imshow("Vision Classifier Simple", debug_vis)
        if red_mask is not None:
            cv2.imshow("RED_MASK", cv2.resize(red_mask, (320, 240)))
        if blue_mask is not None:
            cv2.imshow("BLUE_MASK", cv2.resize(blue_mask, (320, 240)))
        if valid_mask is not None:
            cv2.imshow("VALID_MASK", cv2.resize(valid_mask, (320, 240)))

        cv2.waitKey(1)


def main():
    import sys

    rclpy.init()
    node = LidColorClassifierSimple()

    try:
        if "--webcam" in sys.argv:
            node.get_logger().info("Modo webcam local ativado")
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                node.get_logger().error("Falha ao abrir /dev/video0")
                return

            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                node.process_image(frame, is_webcam_test=True)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            cap.release()
            cv2.destroyAllWindows()
        else:
            rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Desligando vision node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
