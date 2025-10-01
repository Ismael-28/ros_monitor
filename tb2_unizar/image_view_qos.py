#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__("image_viewer_qos")

        # Parámetros ROS
        self.declare_parameter("topic", "/image_raw")
        self.declare_parameter("reliability", "reliable")
        self.declare_parameter("depth", 10)
        self.declare_parameter("flip_horizontal", True)
        self.declare_parameter("flip_vertical", False)
        self.declare_parameter("rotate_deg", 0)  # 0, 90, 180, 270
        self.declare_parameter("encoding", "bgr8")

        topic = self.get_parameter("topic").get_parameter_value().string_value
        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        depth = self.get_parameter("depth").get_parameter_value().integer_value
        self.flip_h = self.get_parameter("flip_horizontal").get_parameter_value().bool_value
        self.flip_v = self.get_parameter("flip_vertical").get_parameter_value().bool_value
        self.rotate_deg = self.get_parameter("rotate_deg").get_parameter_value().integer_value
        self.encoding = self.get_parameter("encoding").get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=(ReliabilityPolicy.RELIABLE if reliability.lower() == "reliable"
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, qos_profile
        )
        self.get_logger().info(
            f"Suscrito a {topic} (rel={reliability}, depth={depth}); "
            f"flip_h={self.flip_h}, flip_v={self.flip_v}, rot={self.rotate_deg}°"
        )

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)

            # Rotación (si procede)
            if self.rotate_deg % 360 != 0:
                if self.rotate_deg % 360 == 90:
                    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                elif self.rotate_deg % 360 == 180:
                    img = cv2.rotate(img, cv2.ROTATE_180)
                elif self.rotate_deg % 360 == 270:
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Flips
            if self.flip_h and self.flip_v:
                img = cv2.flip(img, -1)   # horizontal + vertical
            elif self.flip_h:
                img = cv2.flip(img, 1)    # horizontal
            elif self.flip_v:
                img = cv2.flip(img, 0)    # vertical

            cv2.imshow("Image Viewer QoS", img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
