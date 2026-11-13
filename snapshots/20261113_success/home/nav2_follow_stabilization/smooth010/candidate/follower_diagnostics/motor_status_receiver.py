"""Dedicated read-only ROS executor for motor status; no motion interfaces."""
import threading

class MotorStatusReceiver:
    def __init__(self,callback):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from follower_remote_qos import latest_state_qos
        self.error=None;self._stop=threading.Event();self._closed=False
        self.node=rclpy.create_node('local_avoidance_motor_status_receiver')
        self.subscription=self.node.create_subscription(String,'/follower/motor_candidate/status',callback,latest_state_qos())
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self._run,name='motor-status-receiver',daemon=True);self.thread.start()
    def _run(self):
        try:
            while not self._stop.is_set():self.executor.spin_once(timeout_sec=.02)
        except Exception as exc:
            if not self._stop.is_set():self.error=repr(exc)
    def close(self):
        if self._closed:return True
        self._stop.set();self.thread.join(timeout=.5)
        if self.thread.is_alive():self.error='STATUS_RECEIVER_CLOSE_TIMEOUT';return False
        ok=True
        try:self.executor.shutdown(timeout_sec=0.)
        except Exception as exc:self.error=repr(exc);ok=False
        try:self.node.destroy_node()
        except Exception as exc:self.error=repr(exc);ok=False
        self._closed=ok
        return ok
