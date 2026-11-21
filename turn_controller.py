#!/usr/bin/env python3
"""
Turn controller for the line follower (ROS 2 Jazzy).

This node OWNS /cmd_vel. It does two things:

  FORWARD : drive straight at `forward_speed`, with a small heading-hold
            correction so the robot keeps going in the current target direction.
  TURN    : when an angle arrives on /turn_deg, stop moving forward, rotate in
            place until the target heading is reached (using /odom yaw as
            feedback), then go back to FORWARD in the NEW direction.

Send a turn (positive = left / counter-clockwise, negative = right):

    ros2 topic pub --once /turn_deg std_msgs/msg/Float64 "{data: 90.0}"

Run (simulation time on):

    python3 turn_controller.py --ros-args -p use_sim_time:=true
    python3 turn_controller.py --ros-args -p use_sim_time:=true -p forward_speed:=0.15

Parameters:
    forward_speed     (m/s)    default 0.10
    max_turn_speed    (rad/s)  default 1.0
    min_turn_speed    (rad/s)  default 0.15   avoids stalling near the target
    kp_turn                    default 2.0    turn-in-place gain
    kp_heading                 default 1.5    straight-line heading hold gain
    max_heading_corr  (rad/s)  default 0.5    limit for the heading hold
    tolerance_deg     (deg)    default 1.0    "close enough" for a turn
    start_moving      (bool)   default true   drive forward right away
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def normalize_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class TurnController(Node):
    FORWARD = 'FORWARD'
    TURNING = 'TURNING'

    def __init__(self):
        super().__init__('turn_controller')

        self.forward_speed = self.declare_parameter('forward_speed', 0.10).value
        self.max_turn_speed = self.declare_parameter('max_turn_speed', 1.0).value
        self.min_turn_speed = self.declare_parameter('min_turn_speed', 0.15).value
        self.kp_turn = self.declare_parameter('kp_turn', 2.0).value
        self.kp_heading = self.declare_parameter('kp_heading', 1.5).value
        self.max_heading_corr = self.declare_parameter('max_heading_corr', 0.5).value
        self.tolerance = math.radians(self.declare_parameter('tolerance_deg', 1.0).value)
        self.moving = self.declare_parameter('start_moving', True).value

        self.yaw = None            # latest yaw from odometry
        self.target_yaw = None     # heading we want to travel along
        self.state = self.FORWARD

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Float64, '/turn_deg', self.on_turn, 10)
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.target_yaw is None:
            self.target_yaw = self.yaw

    def on_turn(self, msg):
        if self.target_yaw is None:
            self.get_logger().warn('No /odom yet, ignoring turn command')
            return
        delta = math.radians(msg.data)
        self.target_yaw = normalize_angle(self.target_yaw + delta)
        self.state = self.TURNING
        self.get_logger().info(
            f'Turning {msg.data:.1f} deg -> new heading {math.degrees(self.target_yaw):.1f} deg'
        )

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.yaw is None or self.target_yaw is None:
            return

        err = normalize_angle(self.target_yaw - self.yaw)
        cmd = Twist()

        if self.state == self.TURNING:
            if abs(err) <= self.tolerance:
                self.state = self.FORWARD
                self.get_logger().info('Turn done, going forward in the new direction')
            else:
                w = clamp(self.kp_turn * err, -self.max_turn_speed, self.max_turn_speed)
                if abs(w) < self.min_turn_speed:
                    w = math.copysign(self.min_turn_speed, err)
                cmd.angular.z = w          # linear.x stays 0: turn in place

        if self.state == self.FORWARD:
            if self.moving:
                cmd.linear.x = self.forward_speed
            # Heading hold: keeps the robot driving along target_yaw.
            cmd.angular.z = clamp(
                self.kp_heading * err, -self.max_heading_corr, self.max_heading_corr
            )

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = TurnController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop the robot
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
