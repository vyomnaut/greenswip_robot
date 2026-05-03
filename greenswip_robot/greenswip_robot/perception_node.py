#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('show_debug', False)
        self.show_debug = self.get_parameter('show_debug').value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/box_detection', 10)
        self.get_logger().info('PerceptionNode started – waiting for /camera/image_raw')

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return
        h, w = frame.shape[:2]
        detection = self._detect_box(frame, w, h)
        out = Float32MultiArray()
        out.data = detection
        self.pub.publish(out)
        if self.show_debug:
            self._draw_debug(frame, detection, w, h)

    def _detect_box(self, frame, w, h):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Dark red / maroon box (as rendered by ogre in Gazebo)
        # Covers both bright red and dark red/maroon
        mask1 = cv2.inRange(hsv, np.array([0,   60, 40]),  np.array([15,  255, 200]))
        mask2 = cv2.inRange(hsv, np.array([165, 60, 40]),  np.array([180, 255, 200]))
        mask  = cv2.bitwise_or(mask1, mask2)

        # Exclude blue (sphere) and green (cylinder) and orange (capsule)
        blue_mask   = cv2.inRange(hsv, np.array([100,60,40]), np.array([140,255,255]))
        green_mask  = cv2.inRange(hsv, np.array([40, 60,40]), np.array([90, 255,255]))
        orange_mask = cv2.inRange(hsv, np.array([10, 60,40]), np.array([30, 255,255]))
        exclude = cv2.bitwise_or(cv2.bitwise_or(blue_mask, green_mask), orange_mask)
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(exclude))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best, best_area = None, 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue
            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04*peri, True)
            if not (4 <= len(approx) <= 7):
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if not (0.3 < bw/(bh+1e-6) < 3.0):
                continue
            if area > best_area:
                best_area = area
                best = cnt

        if best is None:
            return [0.0, 0.0, 0.0, 0.0]

        M = cv2.moments(best)
        if M['m00'] == 0:
            return [0.0, 0.0, 0.0, 0.0]

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        cx_norm   = (cx - w/2.0) / (w/2.0)
        cy_norm   = (cy - h/2.0) / (h/2.0)
        area_norm = best_area / (w*h)

        self.get_logger().info(f'BOX detected! cx={cx_norm:.2f} area={area_norm:.3f}')
        return [float(cx_norm), float(cy_norm), float(area_norm), 1.0]

    def _draw_debug(self, frame, det, w, h):
        if det[3] > 0.5:
            cx = int((det[0]*w/2.0) + w/2.0)
            cy = int((det[1]*h/2.0) + h/2.0)
            cv2.circle(frame, (cx,cy), 8, (0,255,0), -1)
            cv2.putText(frame, f'BOX err={det[0]:.2f}', (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            cv2.putText(frame, 'NO BOX', (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow('Debug', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
