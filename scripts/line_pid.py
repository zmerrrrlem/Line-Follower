#!/usr/bin/env python3
"""
PID line follower (ROS 2 Jazzy).

    /line_position      (std_msgs/Float32)  <- line_sensor_node.py (m, + = line to the LEFT, nan = lost)
    /line_follow_enable (std_msgs/Bool)     <- true = follow the line, false = stop publishing
    /cmd_vel_line       (geometry_msgs/Twist) -> twist_mux (lowest priority)

The node only publishes while it is enabled, so it never blocks manual
control or the turn node. It knows nothing about the sensor: anything that
publishes /line_position (simulated array, real IR bar) works.

Enable / disable:
    ros2 topic pub --once /line_follow_enable std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once /line_follow_enable std_msgs/msg/Bool "{data: false}"

Tune at runtime:
    ros2 param set /line_pid kp 1.5

Parameters: see config/line_pid.yaml
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LinePid(Node):
    def __init__(self):
        super().__init__('line_pid')

        p = self.declare_parameter
        self.enabled = p('start_enabled', False).value
        self.kp = p('kp', 1.2).value
        self.ki = p('ki', 0.0).value
        self.kd = p('kd', 0.15).value
        self.integral_limit = p('integral_limit', 0.5).value
        self.d_filter = p('d_filter', 0.3).value
        self.half_span = p('half_span', 0.03325).value
        self.base_speed = p('base_speed', 0.10).value
        self.min_speed = p('min_speed', 0.03).value
        self.slow_gain = p('slow_gain', 0.6).value
        self.max_angular = p('max_angular', 1.5).value
        self.lost_timeout = p('lost_timeout', 1.0).value
        self.lost_turn_speed = p('lost_turn_speed', 0.6).value

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.d_term = 0.0
        self.last_side = 1.0          # +1 = line was last seen on the left
        self.lost_since = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_line', 10)
        self.create_subscription(Float32, '/line_position', self.on_position, 10)
        self.create_subscription(Bool, '/line_follow_enable', self.on_enable, 10)

        self.get_logger().info(
            'Line PID ready (' + ('enabled' if self.enabled else 'disabled, send /line_follow_enable') + ')')

    # ------------------------------------------------------------------
    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.d_term = 0.0
        self.lost_since = None

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.reset()
        if not self.enabled:
            self.pub.publish(Twist())   # one stop command; then stay silent
        self.get_logger().info('Line following ' + ('ENABLED' if self.enabled else 'DISABLED'))

    def on_position(self, msg):
        if not self.enabled:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()

        if math.isnan(msg.data):
            # ---- line lost: spin toward the side it was last seen, then give up ----
            if self.lost_since is None:
                self.lost_since = now
                self.integral = 0.0
                self.prev_error = None
            if now - self.lost_since <= self.lost_timeout:
                cmd.angular.z = math.copysign(self.lost_turn_speed, self.last_side)
            # else: zero command = stop
            self.pub.publish(cmd)
            return

        self.lost_since = None
        error = clamp(msg.data / self.half_span, -1.5, 1.5)
        if abs(error) > 0.05:
            self.last_side = math.copysign(1.0, error)

        # ---- PID ----
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        if dt > 0.0:
            self.integral = clamp(self.integral + self.ki * error * dt,
                                  -self.integral_limit, self.integral_limit)
            raw_d = (error - self.prev_error) / dt if self.prev_error is not None else 0.0
            self.d_term = self.d_filter * self.d_term + (1.0 - self.d_filter) * raw_d
        self.prev_error = error
        self.prev_time = now

        w = self.kp * error + self.integral + self.kd * self.d_term
        cmd.angular.z = clamp(w, -self.max_angular, self.max_angular)
        cmd.linear.x = max(self.min_speed,
                           self.base_speed * (1.0 - self.slow_gain * min(abs(error), 1.0)))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = LinePid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None


if __name__ == '__main__':
    main()
