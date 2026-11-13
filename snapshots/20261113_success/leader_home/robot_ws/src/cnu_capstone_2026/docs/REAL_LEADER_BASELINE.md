# Real Leader Baseline (2026-09-03)

This file records the proven real-hardware leader stack that must remain the rollback baseline while integrating the hierarchical convoy code.

## Proven hardware stack

- Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble
- Workspace: `~/robot_ws`
- MD200T serial: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0`
- MD200T baud: `57600`
- controller ID: `1`
- wheel radius: `0.075 m`
- wheel track: `0.560 m`
- max RPM: `100`
- cmd_vel watchdog: `0.5 s`
- STOP packets keep both motor enable bytes at `1`
- actual RPM feedback: PID216, M1=`data[0:2]`, M2=`data[7:9]`

## Sensors / localization

- SLLIDAR A2M12: `256000`, `/scan` ~11-12 Hz
- iAHRS: `115200`, `/imu/data`, frame `imu_link`
- iAHRS gyro-Z deadband: `abs(gz) < 0.02 -> 0`
- EKF: wheel odometry vx/yaw-rate + IMU gyro-Z
- `/odometry/filtered` produced by robot_localization
- only EKF publishes `odom -> base_link`
- shared TF chain: `map -> odom -> base_link -> laser/imu_link`

## Nav2 baseline

- `base_link` is the robot base frame (not `base_footprint`)
- initial safe test limits:
  - `max_vel_x: 0.10`
  - `max_vel_theta: 0.20`
  - `max_speed_xy: 0.10`
- saved map: `~/robot_ws/maps/leader_map.yaml`
- autonomous navigation with AMCL + Nav2 confirmed working

## Integration rule

Do not replace the proven motor/feedback/EKF/Nav2 stack with Gazebo plugins. The convoy package is added above it through ROS interfaces.

Real leader convoy interface introduced on branch `real-leader-integration`:

- `/leader/odom`: leader pose in the shared `map` frame + filtered twist
- `/leader/path`: sampled leader trajectory in the shared `map` frame

The original Gazebo simulation remains recoverable on branch `sim-success-2026-06-23`.
