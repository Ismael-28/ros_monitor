#!/usr/bin/env python3
import sys
import argparse
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ------------------------------
# Utilidades
# ------------------------------
def ros_time_to_float(t):
    return t.sec + t.nanosec * 1e-9


def qos_best_effort_keep_last(depth: int = 1) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    qos.history = QoSHistoryPolicy.KEEP_LAST
    return qos


# ------------------------------
# Nodo 1: Recolector de métricas
# ------------------------------
class CameraMetricsCollector(Node):
    def __init__(self):
        super().__init__('camera_metrics_collector')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('window_seconds', 5.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('expected_fps', 5.0)

        topic = self.get_parameter('image_topic').value
        self.window = float(self.get_parameter('window_seconds').value)
        pub_rate = float(self.get_parameter('publish_rate').value)
        self.expected_fps = float(self.get_parameter('expected_fps').value)

        # QoS: descartar frames atrasados (cola mínima y BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, topic, self.cb_image, qos_best_effort_keep_last(1)
        )
        self.pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', qos_best_effort_keep_last(1)
        )

        self.recv_times = deque()
        self.capture_times = deque()
        self.bytes_times = deque()

        # Para evitar acceso antes de asignar
        self.last_width = 0
        self.last_height = 0

        if pub_rate > 0:
            self.timer = self.create_timer(1.0 / pub_rate, self.publish_diag)

        self.get_logger().info(f"Suscrito a {topic}  (expected_fps={self.expected_fps})")

    def now_ros(self):
        # Tiempo ROS en segundos (coherente con msg.header.stamp)
        return self.get_clock().now().nanoseconds * 1e-9

    def prune(self, now_s):
        while self.recv_times and (now_s - self.recv_times[0]) > self.window:
            self.recv_times.popleft()
            self.capture_times.popleft()
            self.bytes_times.popleft()

    def cb_image(self, msg: Image):
        now_s = self.now_ros()
        capture_time = ros_time_to_float(msg.header.stamp)
        instant_latency = (now_s - capture_time) * 1000.0  # ms

        self.get_logger().info(f"[camera_metrics] latency_actual_ms={instant_latency:.2f}")

        # Guardar para estadísticas de fps, jitter, etc.
        self.recv_times.append(now_s)
        self.capture_times.append(capture_time)
        self.bytes_times.append(len(msg.data))
        self.last_width = msg.width
        self.last_height = msg.height
        self.prune(now_s)


    def publish_diag(self):
        n = len(self.recv_times)
        if n < 2:
            return

        # Ventana efectiva cubierta por las muestras actuales
        elapsed = self.recv_times[-1] - self.recv_times[0]
        if elapsed <= 0:
            return

        # FPS observado
        fps = (n - 1) / elapsed

        # Jitter (std de intervalos en ms)
        intervals = [self.recv_times[i+1] - self.recv_times[i] for i in range(n - 1)]
        mean_int = sum(intervals) / len(intervals)
        jitter = (sum((x - mean_int) ** 2 for x in intervals) / len(intervals)) ** 0.5 * 1000.0

        # Latencia media (captura -> recepción) en ms
        diffs = [self.recv_times[i] - self.capture_times[i] for i in range(n)]
        latency = (sum(diffs) / len(diffs)) * 1000.0

        # Bitrate medio (Mbps)
        avg_bytes = sum(self.bytes_times) / len(self.bytes_times)
        bitrate = (avg_bytes * fps * 8) / 1e6

        # --- Pérdidas (métrica 1): respecto a FPS esperado ---
        loss_pct = None
        if self.expected_fps > 0:
            expected_count = max(1.0, self.expected_fps * elapsed)
            loss_pct = max(0.0, (1.0 - (n / expected_count)) * 100.0)

        # --- Pérdidas (métrica 2): por huecos en el tiempo ---
        missed_frames = 0
        loss_pct_estimated = None
        if self.expected_fps > 0:
            T = 1.0 / self.expected_fps
            # Umbral de “hueco” generoso para no contar jitter normal
            gap_thresh = 1.5 * T
            for dt in intervals:
                if dt > gap_thresh:
                    missed_frames += max(0, round(dt / T) - 1)
            total = n + missed_frames
            if total > 0:
                loss_pct_estimated = (missed_frames / total) * 100.0

        # Publicación de diagnóstico
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "camera_metrics"
        status.level = DiagnosticStatus.OK
        status.message = "OK" if fps > 0 else "no frames"

        values = [
            KeyValue(key="fps", value=f"{fps:.2f}"),
            KeyValue(key="latency_ms", value=f"{latency:.2f}"),
            KeyValue(key="jitter_ms", value=f"{jitter:.2f}"),
            KeyValue(key="bitrate_mbps", value=f"{bitrate:.3f}"),
            KeyValue(key="resolution", value=f"{self.last_width}x{self.last_height}"),
            KeyValue(key="window_elapsed_s", value=f"{elapsed:.3f}"),
            KeyValue(key="expected_fps", value=f"{self.expected_fps:.2f}"),
        ]

        if loss_pct is not None:
            values.append(KeyValue(key="loss_pct", value=f"{loss_pct:.2f}"))
        if loss_pct_estimated is not None:
            values.append(KeyValue(key="loss_pct_estimated", value=f"{loss_pct_estimated:.2f}"))
            values.append(KeyValue(key="missed_frames", value=str(missed_frames)))
            values.append(KeyValue(key="received_frames_window", value=str(n)))

        status.values = values
        diag.status.append(status)
        self.pub.publish(diag)


# ------------------------------
# Nodo 2: Visor de métricas
# ------------------------------
class DiagViewer(Node):
    def __init__(self):
        super().__init__('camera_metrics_viewer')
        self.sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.cb, qos_best_effort_keep_last(1)
        )

    def cb(self, msg: DiagnosticArray):
        for st in msg.status:
            if st.name == 'camera_metrics':
                kv = {kv.key: kv.value for kv in st.values}
                self.get_logger().info(
                    "[camera_metrics] "
                    f"fps={kv.get('fps')}  "
                    f"latency_ms={kv.get('latency_ms')}  "
                    f"jitter_ms={kv.get('jitter_ms')}  "
                    f"bitrate_mbps={kv.get('bitrate_mbps')}  "
                    f"resolution={kv.get('resolution')}  "
                    f"loss_pct={kv.get('loss_pct', 'NA')}  "
                    f"loss_pct_estimated={kv.get('loss_pct_estimated', 'NA')}  "
                    f"missed_frames={kv.get('missed_frames', 'NA')}"
                )


# ------------------------------
# main: lanzar uno u ambos nodos
# ------------------------------
def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Camera metrics collector + viewer (ROS 2, rclpy)"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--collector", action="store_true", help="Lanza solo el recolector")
    group.add_argument("--viewer", action="store_true", help="Lanza solo el visor")
    # Sin flags -> lanza ambos
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])

    rclpy.init()
    nodes = []

    # Por defecto: ambos. Con flag, solo el elegido.
    if args.viewer and not args.collector:
        nodes.append(DiagViewer())
    elif args.collector and not args.viewer:
        nodes.append(CameraMetricsCollector())
    else:
        nodes.append(CameraMetricsCollector())
        nodes.append(DiagViewer())

    try:
        # Executor multihilo para que ambos nodos procesen callbacks fluidamente
        executor = MultiThreadedExecutor()
        for n in nodes:
            executor.add_node(n)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
