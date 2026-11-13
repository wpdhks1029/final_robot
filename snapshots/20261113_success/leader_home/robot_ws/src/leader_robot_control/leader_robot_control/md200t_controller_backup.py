#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ==============================
# MD200T Protocol
# ==============================

RMID = 0xB7
TMID = 0xB8
CONTROLLER_ID = 0x01
PID_PNT_VEL_CMD = 0xCF


def checksum(packet):
    return (-sum(packet)) & 0xFF


def make_velocity_packet(m1_rpm, m2_rpm):
    """
    MD200T velocity command packet.

    IMPORTANT:
    This function only creates the packet.
    It does NOT transmit anything.
    """

    m1_rpm = max(-32768, min(32767, int(m1_rpm)))
    m2_rpm = max(-32768, min(32767, int(m2_rpm)))

    data = [
        0x01,  # MOT1 command ENABLE
    ]

    data.extend(
        struct.pack("<h", m1_rpm)
    )

    data.append(
        0x01  # MOT2 command ENABLE
    )

    data.extend(
        struct.pack("<h", m2_rpm)
    )

    data.append(
        0x00  # no return request
    )

    packet = [
        RMID,
        TMID,
        CONTROLLER_ID,
        PID_PNT_VEL_CMD,
        0x07,
        *data,
    ]

    packet.append(
        checksum(packet)
    )

    return bytes(packet)


class MD200TController(Node):

    def __init__(self):
        super().__init__('md200t_controller')

        # ==============================
        # Parameters
        # ==============================

        self.declare_parameter(
            'wheel_radius',
            0.075
        )

        self.declare_parameter(
            'wheel_track',
            0.400
        )

        self.declare_parameter(
            'max_rpm',
            100
        )

        self.declare_parameter(
            'dry_run',
            True
        )

        self.wheel_radius = (
            self.get_parameter('wheel_radius')
            .get_parameter_value()
            .double_value
        )

        self.wheel_track = (
            self.get_parameter('wheel_track')
            .get_parameter_value()
            .double_value
        )

        self.max_rpm = (
            self.get_parameter('max_rpm')
            .get_parameter_value()
            .integer_value
        )

        self.dry_run = (
            self.get_parameter('dry_run')
            .get_parameter_value()
            .bool_value
        )

        # ==============================
        # /cmd_vel subscriber
        # ==============================

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ==============================
        # Startup information
        # ==============================

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'Leader Robot MD200T Controller'
        )

        self.get_logger().info(
            f'DRY_RUN = {self.dry_run}'
        )

        self.get_logger().info(
            f'wheel_radius = {self.wheel_radius:.3f} m'
        )

        self.get_logger().info(
            f'wheel_track = {self.wheel_track:.3f} m'
        )

        self.get_logger().info(
            f'max_rpm = {self.max_rpm}'
        )

        self.get_logger().info(
            'NO SERIAL TRANSMISSION IMPLEMENTED'
        )

        self.get_logger().info(
            '================================'
        )

        stop_packet = make_velocity_packet(
            0,
            0
        )

        self.get_logger().info(
            'STOP packet = '
            + stop_packet.hex(' ')
        )


    def cmd_vel_callback(self, msg):

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # ==============================
        # Differential-drive kinematics
        # ==============================

        v_left = (
            linear_x
            - angular_z
            * self.wheel_track
            / 2.0
        )

        v_right = (
            linear_x
            + angular_z
            * self.wheel_track
            / 2.0
        )

        # ==============================
        # m/s -> RPM
        # ==============================

        left_rpm = (
            v_left
            / (
                2.0
                * math.pi
                * self.wheel_radius
            )
            * 60.0
        )

        right_rpm = (
            v_right
            / (
                2.0
                * math.pi
                * self.wheel_radius
            )
            * 60.0
        )

        # ==============================
        # RPM limit
        # ==============================

        left_rpm = max(
            -self.max_rpm,
            min(
                self.max_rpm,
                left_rpm
            )
        )

        right_rpm = max(
            -self.max_rpm,
            min(
                self.max_rpm,
                right_rpm
            )
        )

        m1_rpm = int(
            round(left_rpm)
        )

        m2_rpm = int(
            round(right_rpm)
        )

        # ==============================
        # MD200T packet
        # ==============================

        packet = make_velocity_packet(
            m1_rpm,
            m2_rpm
        )

        checksum_ok = (
            sum(packet) & 0xFF
        ) == 0

        self.get_logger().info(
            '-----------------------------'
        )

        self.get_logger().info(
            f'cmd_vel: '
            f'linear={linear_x:.3f} m/s, '
            f'angular={angular_z:.3f} rad/s'
        )

        self.get_logger().info(
            f'M1 LEFT  = {m1_rpm} rpm'
        )

        self.get_logger().info(
            f'M2 RIGHT = {m2_rpm} rpm'
        )

        self.get_logger().info(
            'PACKET = '
            + packet.hex(' ')
        )

        self.get_logger().info(
            f'CHECKSUM = {checksum_ok}'
        )

        if self.dry_run:

            self.get_logger().info(
                'DRY-RUN: packet NOT transmitted'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MD200TController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
