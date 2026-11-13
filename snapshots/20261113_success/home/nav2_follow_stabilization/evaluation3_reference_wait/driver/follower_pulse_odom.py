"""Optional measured-feedback observer; importing opens neither ROS nor serial."""
import math


def fill_odom(message,core,stamp):
    message.header.stamp=stamp
    message.header.frame_id='follower/odom';message.child_frame_id='follower/base_link'
    message.pose.pose.position.x=core.odom_x;message.pose.pose.position.y=core.odom_y
    message.pose.pose.orientation.z=math.sin(core.odom_yaw/2)
    message.pose.pose.orientation.w=math.cos(core.odom_yaw/2)
    message.twist.twist.linear.x=core.odom_v;message.twist.twist.angular.z=core.odom_w
    message.pose.covariance[0]=message.pose.covariance[7]=.05
    message.pose.covariance[35]=.1
    message.twist.covariance[0]=message.twist.covariance[7]=.02
    message.twist.covariance[35]=.05
    return message


class Observer:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        self.ros=rclpy;self.message_type=Odometry
        rclpy.init(args=[])
        self.node=rclpy.create_node('follower_ground_pulse')
        try:
            if self.node.resolve_topic_name('/follower/odom_raw')!='/follower/odom_raw':
                raise RuntimeError('Odom remap rejected')
            import time
            until=time.monotonic()+1.5
            while time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=.1)
            if self.node.get_publishers_info_by_topic('/follower/odom_raw'):
                raise RuntimeError('Existing odom_raw publisher: stop its owner first')
            self.pub=self.node.create_publisher(Odometry,'/follower/odom_raw',20)
            self.last_count=0
        except BaseException:
            self.close();raise

    def publish(self,core):
        if core.valid_count==self.last_count:return False
        self.last_count=core.valid_count
        self.pub.publish(fill_odom(self.message_type(),core,self.node.get_clock().now().to_msg()))
        return True

    def close(self):
        self.node.destroy_node()
        if self.ros.ok():self.ros.shutdown()
