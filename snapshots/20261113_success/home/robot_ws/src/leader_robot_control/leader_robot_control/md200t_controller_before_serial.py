#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ============================================
# MD200T protocol
# ============================================

RMID = 0xB7
TMID = 0xB8
PID_PNT_VEL_CMD = 0xCF


def checksum(packet):
    return (-sum(packet)) & 0xFF


def make_velocity_packet(controller_id, m1_rpm, m2_rpm):

    m1_rpm = max(-32768, min(32767, int(m1_rpm)))
    m2_rpm = max(-32768, min(32767, int(m2_rpm)))

    data = [
        0x01,                       # MOT1 command ENABLE
        *struct.pack("<h", m1_rpm),

        0x01,                       # MOT2 command ENABLE
        *struct.pack("<h", m2_rpm),

        0x00,                       # no return-data request
    ]

    packet = [
        RMID,
        TMID,
        controller_id,
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

        # ============================================
        # Robot parameters
        # ============================================

        self.declare_parameter(
            'wheel_radius',
            0.075
        )

        self.declare_parameter(
            'wheel_track',
            0.560
        )

        self.declare_parameter(
            'max_rpm',
            100
        )

        # ============================================
        # Direction parameters
        # ============================================

        self.declare_parameter(
            'invert_left',
            False
        )

        self.declare_parameter(
            'invert_right',
            False
        )

        # ============================================
        # Watchdog
        # ============================================

        self.declare_parameter(
            'cmd_vel_timeout',
            0.5
        )

        # ============================================
        # MD200T communication parameters
        # ============================================

        self.declare_parameter(
            'serial_port',
            '/dev/ttyUSB0'
        )

        self.declare_parameter(
            'baudrate',
            57600
        )

        self.declare_parameter(
            'controller_id',
            1
        )

        # ============================================
        # Safety
        # ============================================

        self.declare_parameter(
            'dry_run',
            True
        )

        # ============================================
        # Read parameters
        # ============================================

        self.wheel_radius = (
            self.get_parameter(
                'wheel_radius'
            ).value
        )

        self.wheel_track = (
            self.get_parameter(
                'wheel_track'
            ).value
        )

        self.max_rpm = (
            self.get_parameter(
                'max_rpm'
            ).value
        )

        self.invert_left = (
            self.get_parameter(
                'invert_left'
            ).value
        )

        self.invert_right = (
            self.get_parameter(
                'invert_right'
            ).value
        )

        self.cmd_vel_timeout = (
            self.get_parameter(
                'cmd_vel_timeout'
            ).value
        )

        self.serial_port = (
            self.get_parameter(
                'serial_port'
            ).value
        )

        self.baudrate = (
            self.get_parameter(
                'baudrate'
            ).value
        )

        self.controller_id = (
            self.get_parameter(
                'controller_id'
            ).value
        )

        self.dry_run = (
            self.get_parameter(
                'dry_run'
            ).value
        )

        # ============================================
        # Internal state
        # ============================================

        self.last_cmd_time = None

        self.watchdog_active = True

        # ============================================
        # Subscriber
        # ============================================

        self.subscription = (
            self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
        )

        # ============================================
        # Watchdog timer
        # ============================================

        self.timer = self.create_timer(
            0.1,
            self.watchdog_callback
        )

        # ============================================
        # Startup info
        # ============================================

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'Leader Robot MD200T Controller'
        )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'DRY_RUN = {self.dry_run}'
        )

        self.get_logger().info(
            f'wheel_radius = '
            f'{self.wheel_radius:.3f} m'
        )

        self.get_logger().info(
            f'wheel_track = '
            f'{self.wheel_track:.3f} m'
        )

        self.get_logger().info(
            f'max_rpm = {self.max_rpm}'
        )

        self.get_logger().info(
            f'cmd_vel_timeout = '
            f'{self.cmd_vel_timeout:.2f} sec'
        )

        self.get_logger().info(
            f'invert_left = '
            f'{self.invert_left}'
        )

        self.get_logger().info(
            f'invert_right = '
            f'{self.invert_right}'
        )

        self.get_logger().info(
            f'serial_port = '
            f'{self.serial_port}'
        )

        self.get_logger().info(
            f'baudrate = '
            f'{self.baudrate}'
        )

        self.get_logger().info(
            f'controller_id = '
            f'{self.controller_id}'
        )

        self.get_logger().info(
            'SERIAL TRANSMISSION DISABLED'
        )

        self.get_logger().info(
            '================================'
        )

        self.print_packet(
            'STARTUP STOP',
            0,
            0
        )

    # ================================================
    # Differential drive kinematics
    # ================================================

    def calculate_rpm(
        self,
        linear_x,
        angular_z
    ):

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

        # ============================================
        # RPM limit
        # ============================================

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

        m1 = int(
            round(left_rpm)
        )

        m2 = int(
            round(right_rpm)
        )

        # ============================================
        # Motor direction correction
        # ============================================

        if self.invert_left:
            m1 = -m1

        if self.invert_right:
            m2 = -m2

        return m1, m2

    # ================================================
    # Packet output
    # ================================================

    def print_packet(
        self,
        title,
        m1,
        m2
    ):

        packet = make_velocity_packet(
            self.controller_id,
            m1,
            m2
        )

        checksum_ok = (
            sum(packet) & 0xFF
        ) == 0

        self.get_logger().info(
            f'{title}: '
            f'M1={m1} rpm, '
            f'M2={m2} rpm'
        )

        self.get_logger().info(
            'PACKET = '
            + packet.hex(' ')
        )

        self.get_logger().info(
            f'CHECKSUM = '
            f'{checksum_ok}'
        )

        if self.dry_run:
            self.get_logger().info(
                'DRY-RUN: packet NOT transmitted'
            )

    # ================================================
    # /cmd_vel callback
    # ================================================

    def cmd_vel_callback(
        self,
        msg
    ):

        self.last_cmd_time = (
            self.get_clock().now()
        )

        if self.watchdog_active:

            self.watchdog_active = False

            self.get_logger().info(
                'WATCHDOG cleared: '
                'cmd_vel received'
            )

        m1, m2 = self.calculate_rpm(
            msg.linear.x,
            msg.angular.z
        )

        self.get_logger().info(
            '-----------------------------'
        )

        self.get_logger().info(
            f'cmd_vel: '
            f'linear={msg.linear.x:.3f} m/s, '
            f'angular={msg.angular.z:.3f} rad/s'
        )

        self.print_packet(
            'COMMAND',
            m1,
            m2
        )

    # ================================================
    # Watchdog
    # ================================================

    def watchdog_callback(self):

        if self.last_cmd_time is None:
            return

        now = (
            self.get_clock().now()
        )

        elapsed = (
            now
            - self.last_cmd_time
        ).nanoseconds / 1e9

        if (
            elapsed
            > self.cmd_vel_timeout
            and not self.watchdog_active
        ):

            self.watchdog_active = True

            self.get_logger().warning(
                f'WATCHDOG: '
                f'no cmd_vel for '
                f'{elapsed:.2f} sec'
            )

            self.print_packet(
                'WATCHDOG STOP',
                0,
                0
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
