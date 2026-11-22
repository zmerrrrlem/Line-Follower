#!/usr/bin/env python3
"""
Turn node (ROS 2 Jazzy).

Rotates the robot in place by a relative angle, then goes silent again.

    /turn_deg      (std_msgs/Float64)    <- relative turn in degrees, + = left, - = right
    /odom          (nav_msgs/Odometry)   <- yaw feedback
    /cmd_vel_turn  (geometry_msgs/Twist) -> twist_mux (highest priority)

It publishes ONLY while turning (plus one zero command when the turn is done),
so manual control and the line follower work normally the rest of the time.
While it is turning, twist_mux ignores the other sources.

Usage:
    ros2 topic pub --once /turn_deg std_msgs/msg/Float64 "{data: 90.0}"

Parameters (set with --ros-args -p name:=value):
    max_turn_speed  (rad/s)  default 1.0
    min_turn_speed  (rad/s)  default 0.15   avoids stalling near the target
    kp_turn                  default 2.0
    tolerance_deg   (deg)    default 1.0
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TurnNode(Node):
    def __init__(self):
        super().__init__('turn_node')

        self.max_turn_speed = self.declare_parameter('max_turn_speed', 1.0).value
        self.min_turn_speed = self.declare_parameter('min_turn_speed', 0.15).value
        self.kp_turn = self.declare_parameter('kp_turn', 2.0).value
        self.tolerance = math.radians(self.declare_parameter('tolerance_deg', 1.0).value)

        self.yaw = None
        self.target_yaw = None
        self.turning = False

        self.pub = self.create_publisher(Twist, '/cmd_vel_turn', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Float64, '/turn_deg', self.on_turn, 10)
        self.create_timer(0.05, self.control_loop) 

        self.get_logger().info(
            'Turn node ready: ros2 topic pub --once /turn_deg std_msgs/msg/Float64 "{data: 90.0}"')

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_turn(self, msg):
        if self.yaw is None:
            self.get_logger().warn('No /odom yet, ignoring turn command')
            return
        # A new request while turning is added to the current target.
        base = self.target_yaw if self.turning else self.yaw
        self.target_yaw = normalize_angle(base + math.radians(msg.data))
        self.turning = True
        self.get_logger().info(
            f'Turning {msg.data:.1f} deg -> target heading {math.degrees(self.target_yaw):.1f} deg')

    def control_loop(self):
        if not self.turning or self.yaw is None:
            return                                   # silent: do not touch /cmd_vel_turn

        err = normalize_angle(self.target_yaw - self.yaw)
        cmd = Twist()
        if abs(err) <= self.tolerance:
            self.turning = False
            self.pub.publish(cmd)                    # one stop command
            self.get_logger().info('Turn done')
            return

        w = clamp(self.kp_turn * err, -self.max_turn_speed, self.max_turn_speed)
        if abs(w) < self.min_turn_speed:
            w = math.copysign(self.min_turn_speed, err)
        cmd.angular.z = w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = TurnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
