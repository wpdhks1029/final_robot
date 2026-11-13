import ast,queue,sys,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
from motor_status_receiver import MotorStatusReceiver
from run_local_avoidance_trial import motor_problem

class Tests(unittest.TestCase):
    def setup_modules(self,fail=False):
        class Node:
            def create_subscription(self,k,topic,cb,qos):self.callback=cb;self.topic=topic;return cb
            def destroy_node(self):self.destroyed=True
        node=Node()
        class Executor:
            def __init__(self):self.q=queue.Queue()
            def add_node(self,n):self.node=n
            def spin_once(self,timeout_sec):
                if fail:raise RuntimeError('receiver transport failure')
                try:item=self.q.get(timeout=timeout_sec)
                except queue.Empty:return
                self.node.callback(item)
            def shutdown(self,timeout_sec):pass
        return node,{'rclpy':S(create_node=lambda name:node),'rclpy.executors':S(SingleThreadedExecutor=Executor),'std_msgs':S(),'std_msgs.msg':S(String=object),'follower_remote_qos':S(latest_state_qos=lambda:1)}
    def test_status_arrives_on_own_executor_while_operator_is_busy(self):
        node,modules=self.setup_modules();received=[];event=threading.Event()
        def callback(msg):received.append((msg,time.monotonic(),threading.current_thread().ident));event.set()
        with patch.dict(sys.modules,modules):
            r=MotorStatusReceiver(callback)
            try:
                r.executor.q.put('fresh status')
                self.assertTrue(event.wait(.5))
                self.assertNotEqual(received[-1][2],threading.current_thread().ident)
                self.assertEqual(node.topic,'/follower/motor_candidate/status')
            finally:self.assertTrue(r.close())
            self.assertTrue(r.close());self.assertFalse(r.thread.is_alive());self.assertTrue(node.destroyed)
    def test_receiver_failure_is_reported_instead_of_hidden(self):
        node,modules=self.setup_modules(fail=True)
        with patch.dict(sys.modules,modules):
            r=MotorStatusReceiver(lambda msg:None);r.thread.join(.5)
            self.assertIn('receiver transport failure',r.error);self.assertTrue(r.close())
    def test_actual_stale_status_still_rejected_at_same_limit(self):
        self.assertEqual(motor_problem(({},1.),1.301,360),'MOTOR_STATUS_MISSING_OR_STALE')
    def test_receiver_has_no_motion_publishers_or_services(self):
        tree=ast.parse(Path(__file__).with_name('motor_status_receiver.py').read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls&{'create_publisher','create_client','call_async'})

if __name__=='__main__':unittest.main()
