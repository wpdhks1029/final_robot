"""TEST position EKF lifecycle attached to the single-owner measurement session."""
import copy
from pathlib import Path
import subprocess
import time

from follower_comparison_observer import ComparisonObserver

OUTPUT='/follower/odometry/position_filtered_test'


def ekf_command():
    return ['/opt/ros/humble/lib/robot_localization/ekf_node','--ros-args',
            '-r','__node:=ekf','-r','__ns:=/follower_position_ekf_test',
            '--params-file',str(Path(__file__).with_name('follower_position_ekf_test.yaml')),
            '-r','odometry/filtered:='+OUTPUT,'-r','tf:=/tf','-r','tf_static:=/tf_static']


class EkfComparisonObserver(ComparisonObserver):
    def __init__(self):
        self.child=None;self.child_log=None;self.imu_count=0
        super().__init__()
        try:
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
            from geometry_msgs.msg import TransformStamped
            from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
            from rclpy.qos import QoSProfile,ReliabilityPolicy
            if self.node.get_publishers_info_by_topic(OUTPUT):raise RuntimeError('Existing TEST EKF publisher')
            if self.node.get_publishers_info_by_topic('/follower/imu_position_test'):
                raise RuntimeError('Existing TEST gyro relay')
            self.imu_pub=self.node.create_publisher(Imu,'/follower/imu_position_test',20)
            qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subs.append(self.node.create_subscription(Imu,'/follower/imu/data',self.relay,qos))
            self.rows[OUTPUT]=[]
            self.subs.append(self.node.create_subscription(Odometry,OUTPUT,lambda m:self.receive(OUTPUT,m),qos))
            self.mount=StaticTransformBroadcaster(self.node)
            tf=TransformStamped();tf.header.stamp=self.node.get_clock().now().to_msg()
            tf.header.frame_id='follower_position_preview/base_link';tf.child_frame_id='follower_position_preview/imu_link'
            tf.transform.translation.z=.16;tf.transform.rotation.w=1.
            self.mount.sendTransform(tf)
            log_path=self.path.with_suffix('.ekf.log')
            self.child_log=log_path.open('x')
            self.child=subprocess.Popen(ekf_command(),stdout=self.child_log,stderr=subprocess.STDOUT)
            print('TEST_EKF_LOG='+str(log_path),flush=True)
        except BaseException:
            self.close();raise

    def relay(self,m):
        copied=copy.deepcopy(m);copied.header.frame_id='follower_position_preview/imu_link'
        self.imu_pub.publish(copied);self.imu_count+=1

    def publish(self,core):
        if self.child.poll() is not None:raise RuntimeError('TEST EKF exited; stopping measurement session')
        return super().publish(core)

    def comparison_ready(self):
        return (self.child.poll() is None and self.imu_count>=10
                and len(self.rows.get(OUTPUT,[]))>=10)

    def finish_capture(self):
        if self.finished:return
        print('gyro_relay_received='+str(self.imu_count),flush=True)
        super().finish_capture()

    def close(self):
        child=getattr(self,'child',None)
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        logfile=getattr(self,'child_log',None)
        if logfile is not None:logfile.close();self.child_log=None
        super().close()
