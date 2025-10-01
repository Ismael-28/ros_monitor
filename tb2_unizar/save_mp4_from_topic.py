# save_mp4_from_topic.py
import argparse, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Mp4Recorder(Node):
    def __init__(self, topic, outfile, fps, codec):
        super().__init__('mp4_recorder')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.out = None
        self.outfile = outfile
        self.fps = fps
        self.codec = codec

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.out is None:
            fourcc = cv2.VideoWriter_fourcc(*self.codec)
            h, w = frame.shape[:2]
            self.out = cv2.VideoWriter(self.outfile, fourcc, self.fps, (w, h))
            if not self.out.isOpened():
                self.get_logger().error("No se pudo abrir VideoWriter. ¿FFmpeg disponible?")
                rclpy.shutdown()
                return

        self.out.write(frame)

    def destroy_node(self):
        if self.out:
            self.out.release()
        super().destroy_node()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/image_raw')
    parser.add_argument('--outfile', default='output.mp4')
    parser.add_argument('--fps', type=float, default=5.0)
    parser.add_argument('--codec', default='mp4v', help="mp4v (MPEG-4) o avc1 (H.264 si tu OpenCV lo soporta)")
    args = parser.parse_args()

    rclpy.init()
    node = Mp4Recorder(args.topic, args.outfile, args.fps, args.codec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
