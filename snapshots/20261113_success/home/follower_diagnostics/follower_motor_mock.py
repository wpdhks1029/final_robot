#!/usr/bin/env python3
"""ROS adapter for an in-memory MD200T model only. No physical port implementation.

Manual JSON String envelopes atomically carry version/epoch/frame/stamp/v/w.
This experimental wire contract is NOT connected to the real Twist preview chain.
"""
import json
import math
import re
import struct

from follower_motor_candidate import MotorCore,Config

PREFIX='/follower/motor_mock'


def strict_pairs(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('Duplicate envelope field')
        result[key]=value
    return result


def parse_envelope(text):
    if not isinstance(text,str) or len(text)>2048:raise ValueError('Envelope too large or not text')
    obj=json.loads(text,object_pairs_hook=strict_pairs)
    keys={'version','epoch','frame','stamp_ns','v','w'}
    if not isinstance(obj,dict) or set(obj)!=keys:raise ValueError('Unexpected envelope fields')
    if type(obj['version']) is not int or obj['version']!=1:raise ValueError('Unknown version')
    if not isinstance(obj['epoch'],str) or not re.fullmatch('[0-9a-f]{16}',obj['epoch']):
        raise ValueError('Invalid epoch')
    if obj['frame']!='follower/base_link':raise ValueError('Invalid command frame')
    if type(obj['stamp_ns']) is not int or obj['stamp_ns']<=0:raise ValueError('Invalid stamp')
    for key in ('v','w'):
        if type(obj[key]) not in (int,float) or not math.isfinite(obj[key]):
            raise ValueError('Invalid velocity')
    return obj


class MemoryTransport:
    """Instant RPM echo model, NOT acceleration/braking/stall/physical feedback."""
    timeout=0
    write_timeout=.01

    def __init__(self,driver_id=1):
        self.driver_id=driver_id;self.rx=bytearray()
        self.feedback_enabled=True;self.closed=False
        self.rpm=(0,0);self.writes=0

    @property
    def in_waiting(self):return len(self.rx)

    def reset_input_buffer(self):self.rx.clear()

    def read(self,n):
        raw=bytes(self.rx[:n]);del self.rx[:n];return raw

    def write(self,packet):
        if self.closed:raise OSError('Memory transport is closed')
        if (sum(packet)&255 or packet[:3]!=bytes([0xB7,0xB8,self.driver_id])):
            raise ValueError('Malformed simulated packet')
        if len(packet)==7 and packet[3:6]==bytes([4,1,216]):
            pass
        elif (len(packet)==13 and packet[3:5]==bytes([207,7])
              and packet[5]==1 and packet[8]==1 and packet[11]==1):
            self.rpm=(struct.unpack_from('<h',packet,6)[0],struct.unpack_from('<h',packet,9)[0])
        else:raise ValueError('Unsupported simulated packet')
        self.writes+=1
        if self.feedback_enabled:
            left,right=self.rpm
            payload=struct.pack('<hBihBi',left,0,0,right,0,0)
            raw=bytes([0xB8,0xB7,self.driver_id,216,14])+payload
            self.rx.extend(raw+bytes([(-sum(raw))&255]))
        return len(packet)

    def close(self):self.closed=True;self.rx.clear()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,SetBool
    from rcl_interfaces.msg import ParameterDescriptor

    class MockNode(Node):
        def __init__(self):
            super().__init__('follower_motor_mock')
            self.declare_parameter('enable_simulated_motion',False,
                                   ParameterDescriptor(read_only=True))
            enabled=self.get_parameter('enable_simulated_motion').value
            if type(enabled) is not bool:raise ValueError('enable_simulated_motion must be bool')
            topics=[PREFIX+'/manual_command',PREFIX+'/status',PREFIX+'/odom']
            services=[PREFIX+'/arm',PREFIX+'/disarm',PREFIX+'/set_feedback_enabled']
            if any(self.resolve_topic_name(t)!=t for t in topics):
                raise RuntimeError('Mock topic remapping is not permitted')
            if any(self.resolve_service_name(s)!=s for s in services):
                raise RuntimeError('Mock service remapping is not permitted')
            self.transport=MemoryTransport()
            self.core=MotorCore(self.transport,dry_run=False,allow_motion=enabled,config=Config())
            self.rejected=0;self.accepted=0;self.last_publish=0.
            self.status_pub=self.create_publisher(String,PREFIX+'/status',10)
            self.odom_pub=self.create_publisher(Odometry,PREFIX+'/odom',10)
            self.command_sub=self.create_subscription(String,PREFIX+'/manual_command',self.command_cb,10)
            self.arm_srv=self.create_service(Trigger,PREFIX+'/arm',self.arm_cb)
            self.disarm_srv=self.create_service(Trigger,PREFIX+'/disarm',self.disarm_cb)
            self.feedback_srv=self.create_service(SetBool,PREFIX+'/set_feedback_enabled',self.feedback_cb)
            self.timer=self.create_timer(.02,self.tick)
            self.get_logger().info('MEMORY MODEL ONLY: no serial ports, real odom, TF or motor command outputs')

        def command_cb(self,msg):
            try:
                obj=parse_envelope(msg.data)
                ok=self.core.accept_command(obj['v'],obj['w'],frame=obj['frame'],stamp_ns=obj['stamp_ns'],
                    ros_now_ns=self.get_clock().now().nanoseconds,epoch=obj['epoch'])
                if not ok:self.rejected+=1
                else:self.accepted+=1
            except (ValueError,TypeError,KeyError,OverflowError,RecursionError):
                self.rejected+=1;self.core.latch_fault('INVALID_ENVELOPE')

        def arm_cb(self,request,response):
            epoch=self.core.arm(self.core.epoch,self.get_clock().now().nanoseconds)
            response.success=epoch is not None
            response.message=json.dumps({'epoch':epoch,'reason':None if epoch else 'MOCK_LOCK_OR_FEEDBACK_NOT_READY'})
            return response

        def disarm_cb(self,request,response):
            self.core.disarm();response.success=True;response.message='Mock disarmed; old epoch invalidated'
            return response

        def feedback_cb(self,request,response):
            self.transport.feedback_enabled=request.data
            if not request.data:self.transport.rx.clear()
            response.success=True;response.message='Memory feedback enabled' if request.data else 'Memory feedback dropped'
            return response

        def tick(self):
            now=self.get_clock().now()
            before=self.core.valid_count
            self.core.tick(now.nanoseconds)
            if self.core.valid_count!=before:
                odom=Odometry();odom.header.stamp=now.to_msg()
                odom.header.frame_id='follower_motor_mock/odom'
                odom.child_frame_id='follower_motor_mock/base_link'
                odom.pose.pose.position.x=self.core.odom_x;odom.pose.pose.position.y=self.core.odom_y
                odom.pose.pose.orientation.z=math.sin(self.core.odom_yaw/2)
                odom.pose.pose.orientation.w=math.cos(self.core.odom_yaw/2)
                odom.twist.twist.linear.x=self.core.odom_v;odom.twist.twist.angular.z=self.core.odom_w
                # Model-only covariance; not an experimental sensor calibration.
                odom.pose.covariance[0]=odom.pose.covariance[7]=1.
                odom.pose.covariance[35]=1.;odom.twist.covariance[0]=.02;odom.twist.covariance[35]=.05
                self.odom_pub.publish(odom)
            if self.core.clock()-self.last_publish>=.1:
                self.last_publish=self.core.clock()
                status={'mode':'MEMORY_ONLY','simulated_motion_enabled':self.core.allow_motion,
                    'armed':self.core.armed,'fault':self.core.fault,'epoch':self.core.epoch,
                    'feedback_fresh':self.core.feedback_fresh(),'valid_feedback':self.core.valid_count,
                    'measured_rpm':None if self.core.feedback is None else
                        [self.core.feedback['m1_rpm'],self.core.feedback['m2_rpm']],
                    'last_sent_rpm':self.core.last_sent,'model_rpm':self.transport.rpm,
                    'rejected_commands':self.rejected,'accepted_commands':self.accepted,
                    'io_failed':self.core.io_failed,
                    'rx_enabled':self.transport.feedback_enabled}
                self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))

    rclpy.init()
    node=None
    try:node=MockNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.core.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
