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
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("publish_debug", True)

        # Thresholds / ROI
        self.declare_parameter("min_area_ratio", 0.005)  # área mínima (fração da imagem) para aceitar cor
        self.declare_parameter("roi_ymin", 0.20)          # recorta parte superior/inferior (0..1)
        self.declare_parameter("roi_ymax", 0.90)
        self.declare_parameter("roi_xmin", 0.10)
        self.declare_parameter("roi_xmax", 0.90)

        # HSV thresholds
        # Vermelho (Cores fortes, Ignora laranjas e marrons de pele)
        self.declare_parameter("red_h1_low", 0)
        self.declare_parameter("red_h1_high", 5)
        self.declare_parameter("red_h2_low", 170)
        self.declare_parameter("red_h2_high", 179)
        self.declare_parameter("red_s_low", 120)
        self.declare_parameter("red_v_low", 80)

        # Azul (Ignora brancos levemente azulados e sobras)
        self.declare_parameter("blue_h_low", 95)
        self.declare_parameter("blue_h_high", 125)
        self.declare_parameter("blue_s_low", 150)
        self.declare_parameter("blue_v_low", 80)

        self.bridge = CvBridge()

        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.pub_class = self.create_publisher(String, "/product_class", 10)
        self.pub_dbg = self.create_publisher(String, "/product_class_debug", 10)

        self.get_logger().info(f"Vision node OK. Listening on: {image_topic}")

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_image(bgr)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    def process_image(self, bgr, is_webcam_test=False):
        h, w = bgr.shape[:2]

        # ROI crop (ajuda a ignorar fundo)
        roi = self._crop_roi(bgr)

        # Suaviza a imagem sem quebrar as bordas, removendo serrilhados para distância
        blurred = cv2.bilateralFilter(roi, 9, 75, 75)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Retorna a máscara e o polígono detectado (se houver)
        red_mask, red_poly = self._detect_shape(hsv, "red")    # Procura triângulos (3 lados)
        blue_mask, blue_poly = self._detect_shape(hsv, "blue") # Procura quadrados (4 lados)

        label = "unknown"
        if red_poly is not None and len(red_poly) == 3:
            label = "red"
        elif blue_poly is not None and len(blue_poly) == 4:
            label = "blue"

        if not is_webcam_test:
            self.pub_class.publish(String(data=label))

            if bool(self.get_parameter("publish_debug").value):
                dbg = f"Detected: {label} | poly: R={len(red_poly) if red_poly is not None else 0} B={len(blue_poly) if blue_poly is not None else 0}"
                self.pub_dbg.publish(String(data=dbg))

        # --- Debug visual interativo ---
        y0 = int(float(self.get_parameter("roi_ymin").value) * h)
        y1 = int(float(self.get_parameter("roi_ymax").value) * h)
        x0 = int(float(self.get_parameter("roi_xmin").value) * w)
        x1 = int(float(self.get_parameter("roi_xmax").value) * w)

        # Desenha a ROI bounds (verde)
        cv2.rectangle(bgr, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
        
        # Desenha o polígono detectado transladando as coordenadas de volta para a imagem original
        if red_poly is not None and len(red_poly) == 3:
            red_poly_orig = red_poly + [x0, y0]
            cv2.drawContours(bgr, [red_poly_orig], 0, (0, 0, 255), 4) # Vermeho (thick line)
        
        if blue_poly is not None and len(blue_poly) == 4:
            blue_poly_orig = blue_poly + [x0, y0]
            cv2.drawContours(bgr, [blue_poly_orig], 0, (255, 0, 0), 4) # Azul (thick line)

        # Status HUD
        color_bgr = (0, 0, 255) if label == "red" else ((255, 0, 0) if label == "blue" else (128, 128, 128))
        text = "TRIANGLE (RED)" if label == "red" else "SQUARE (BLUE)" if label == "blue" else "UNKNOWN"
        cv2.putText(bgr, f"Class: {text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_bgr, 2)
        
        poly_dbg = f"R_sides: {len(red_poly) if red_poly is not None else 0} | B_sides: {len(blue_poly) if blue_poly is not None else 0}"
        cv2.putText(bgr, poly_dbg, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("Vision Classifier - Debug", bgr)
        
        # Exibe as máscaras em janelas menores para facilitar o tuning
        if is_webcam_test:
            # Re_size mask to 320x240 para não poluir a tela
            cv2.imshow("MASK RED (White is Red)", cv2.resize(red_mask, (320, 240)))
            cv2.imshow("MASK BLUE (White is Blue)", cv2.resize(blue_mask, (320, 240)))
        
        cv2.waitKey(1)

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

    def _detect_shape(self, hsv, color):
        """ Filtra a cor, acha contornos e aproxima polígonos. Retorna (mask, poly). """
        if color == "red":
            h1l, h1h = int(self.get_parameter("red_h1_low").value), int(self.get_parameter("red_h1_high").value)
            h2l, h2h = int(self.get_parameter("red_h2_low").value), int(self.get_parameter("red_h2_high").value)
            sl, vl = int(self.get_parameter("red_s_low").value), int(self.get_parameter("red_v_low").value)
            
            mask1 = cv2.inRange(hsv, np.array([h1l, sl, vl], dtype=np.uint8), np.array([h1h, 255, 255], dtype=np.uint8))
            mask2 = cv2.inRange(hsv, np.array([h2l, sl, vl], dtype=np.uint8), np.array([h2h, 255, 255], dtype=np.uint8))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            hl, hh = int(self.get_parameter("blue_h_low").value), int(self.get_parameter("blue_h_high").value)
            sl, vl = int(self.get_parameter("blue_s_low").value), int(self.get_parameter("blue_v_low").value)
            mask = cv2.inRange(hsv, np.array([hl, sl, vl], dtype=np.uint8), np.array([hh, 255, 255], dtype=np.uint8))

        mask = self._cleanup(mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return mask, None
        
        # Filtra os ruídos menores que ~0.5% e elimina enormes que seriam o fundo ou a borda da tampa (> 40%)
        h, w = hsv.shape[:2]
        img_area = h * w
        min_pixels = float(self.get_parameter("min_area_ratio").value) * img_area
        max_pixels = 0.4 * img_area
        
        best_poly = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_pixels < area < max_pixels and area > max_area:
                
                # Check aspect ratio (triângulos e quadrados do decalque são relativamente simétricos, não esticões)
                x, y, bw, bh = cv2.boundingRect(cnt)
                aspect_ratio = float(bw) / max(1, bh)
                if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                    continue # Rejeita polígonos finos e compridos demais
                
                hull = cv2.convexHull(cnt)
                epsilon = 0.05 * cv2.arcLength(hull, True)
                approx = cv2.approxPolyDP(hull, epsilon, True)
                
                max_area = area
                best_poly = approx
                
        return mask, best_poly

    @staticmethod
    def _cleanup(mask):
        # remove ruído
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask

def main():
    import sys
    rclpy.init()
    node = LidColorClassifier()

    try:
        if "--webcam" in sys.argv:
            node.get_logger().info("--- MODO WEBCAM LOCAL ATIVADO ---")
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                node.get_logger().error("Falha ao abrir /dev/video0")
                return
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                node.process_image(frame, is_webcam_test=True)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            cap.release()
            cv2.destroyAllWindows()
        else:
            # Modo ROS normal
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Desligando Vision Node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

if __name__ == "__main__":
    main()
