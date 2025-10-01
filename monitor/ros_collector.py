# -*- coding: utf-8 -*-
# ros_collector.py
import threading
import queue

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from diagnostic_msgs.msg import DiagnosticArray
    ROS_OK = True
except Exception:
    ROS_OK = False

try:
    from theme import console
except Exception:
    class _DummyConsole:
        def print(self, *a, **k):
            try: print(*a)
            except Exception: pass
    console = _DummyConsole()

if not ROS_OK:
    # 👇 Si ROS no está, no exponemos clase alguna
    CameraDiagCollector = None
else:
    class _DiagSub(Node):
        def __init__(self, queue_put, topic: str = '/diagnostics'):
            super().__init__('camera_diag_for_metrics')
            qos = QoSProfile(depth=5)
            qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            qos.history     = QoSHistoryPolicy.KEEP_LAST
            self.queue_put = queue_put
            self.sub = self.create_subscription(DiagnosticArray, topic, self.cb, qos)

        @staticmethod
        def _kv(status):
            return {kv.key: kv.value for kv in status.values}

        def cb(self, msg: DiagnosticArray):
            for st in msg.status:
                if st.name != 'camera_metrics':
                    continue
                kv = self._kv(st)
                def f(key):
                    v = kv.get(key)
                    try:
                        return float(v) if v not in (None, "") else None
                    except ValueError:
                        return None
                latency = f('latency_ms')
                jitter  = f('jitter_ms')
                loss    = f('loss_pct_estimated') if kv.get('loss_pct_estimated') else f('loss_pct')
                self.queue_put((latency, jitter, loss))

    class CameraDiagCollector(threading.Thread):
        def __init__(self, diagnostics_topic: str = '/diagnostics', interval: float = 1.0):
            super().__init__(daemon=True)
            self.queue = queue.Queue()
            self.interval = interval
            self._topic = diagnostics_topic
            self._exec = None
            self._node = None

        def run(self):
            try:
                rclpy.init()
                self._node = _DiagSub(self.queue.put, self._topic)
                self._exec = MultiThreadedExecutor()
                self._exec.add_node(self._node)
                self._exec.spin()
            except Exception as e:
                console.print(f"[error]Executor ROS detenido: {e}[/error]")
            finally:
                try:
                    if self._exec: self._exec.shutdown()
                except Exception: pass
                try:
                    if self._node: self._node.destroy_node()
                except Exception: pass
                try:
                    rclpy.shutdown()
                except Exception: pass

        def stop(self):
            try:
                if self._exec: self._exec.shutdown()
            except Exception:
                pass

        def get_latest(self):
            try:
                return self.queue.get_nowait()
            except queue.Empty:
                return None
