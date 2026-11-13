"""Synthetic sensor traffic only, isolated ROS domain217 on localhost."""
import os,time,unittest
from sensor_executor import SensorExecutor

@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID')=='217' and os.environ.get('ROS_LOCALHOST_ONLY')=='1','requires isolated localhost domain217')
class Tests(unittest.TestCase):
    def test_eight_sensor_streams_are_serviced_without_starvation(self):
        import rclpy
        from std_msgs.msg import UInt64
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        rclpy.init(args=[]);source=rclpy.create_node('synthetic_sensor_source');sink=rclpy.create_node('synthetic_sensor_sink')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);counts=[0]*8;ages=[[] for _ in range(8)];subs=[];pubs=[]
        def receive(i,msg):counts[i]+=1;ages[i].append((time.monotonic_ns()-msg.data)/1e9)
        for i in range(8):
            topic='/executor_test/sensor_'+str(i)
            pubs.append(source.create_publisher(UInt64,topic,qos));subs.append(sink.create_subscription(UInt64,topic,lambda m,i=i:receive(i,m),qos))
        executor=SensorExecutor(sink)
        try:
            begin=time.monotonic()
            while time.monotonic()-begin<2.:
                for pub in pubs:pub.publish(UInt64(data=time.monotonic_ns()))
                executor.poll()
            self.assertGreater(min(counts),10)
            self.assertLess(max(max(a) for a in ages),.3)
            print('synthetic_counts='+str(counts)+' max_callback_age_s='+str(max(max(a) for a in ages)))
        finally:
            executor.close();source.destroy_node();sink.destroy_node()
            if rclpy.ok():rclpy.shutdown()

    def test_history_retains_measurements_for_map_pose_interpolation(self):
        import rclpy
        from std_msgs.msg import UInt64
        from follower_remote_qos import measurement_history_qos
        from local_stop_turn_go import pose_at
        rclpy.init(args=[]);source=rclpy.create_node('history_sensor_source');sink=rclpy.create_node('history_sensor_sink')
        qos=measurement_history_qos();received=[]
        pub=source.create_publisher(UInt64,'/executor_test/history',qos)
        sub=sink.create_subscription(UInt64,'/executor_test/history',lambda m:received.append(m.data),qos)
        executor=SensorExecutor(sink)
        try:
            self.assertEqual(qos.depth,10)
            ns=100_000_000_000
            for i in range(6):pub.publish(UInt64(data=ns+i*50_000_000))
            deadline=time.monotonic()+.5
            while len(received)<6 and time.monotonic()<deadline:executor.poll()
            self.assertEqual(len(received),6)
            history=[(stamp,1.,(0.,0.,0.,0.,0.)) for stamp in received]
            pose,support,uncertainty=pose_at(history,ns+125_000_000)
            self.assertEqual(support,ns+100_000_000);self.assertEqual(uncertainty,0.)
        finally:
            executor.close();source.destroy_node();sink.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':unittest.main()
