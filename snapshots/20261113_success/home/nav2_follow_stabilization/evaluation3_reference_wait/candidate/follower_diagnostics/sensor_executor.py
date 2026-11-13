"""Keep the operator node registered across bounded callback batches."""
import time

class SensorExecutor:
    def __init__(self,node):
        from rclpy.executors import SingleThreadedExecutor
        self.node=node;self.executor=SingleThreadedExecutor();self.executor.add_node(node)
    def poll(self):
        start=time.monotonic()
        for _ in range(32):
            if time.monotonic()-start>=.010:break
            # Keep identical wait arguments so the executor can continue its
            # ready-callback iterator across batches rather than restarting it.
            self.executor.spin_once(timeout_sec=.001)
    def close(self):
        try:
            self.executor.remove_node(self.node);self.executor.shutdown(timeout_sec=0.)
            return True
        except Exception:return False
