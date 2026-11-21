#!/usr/bin/env python3
"""
Line sensor array for the line follower (ROS 2 Jazzy).

Turns the image of the downward-looking line camera into the readings of an
N-sensor reflectance bar (like a QTR-8):

    /line_camera/image   (sensor_msgs/Image, rgb8)   <- from Gazebo via ros_gz_bridge
    /line_sensors        (std_msgs/Float32MultiArray) -> N values, 1.0 = white floor, 0.0 = black line
    /line_position       (std_msgs/Float32)           -> line position across the bar in metres,
                                                         + = line to the robot's LEFT, - = RIGHT,
                                                         NaN = line lost (no sensor sees black)

Sensor 0 is the left-most sensor (robot's left, +y), sensor N-1 the right-most.

Run:
    python3 line_sensor_node.py --ros-args -p use_sim_time:=true

Parameters:
    n_sensors          default 8       number of sensors on the bar
    spacing            default 0.0095  distance between sensors (m)
    window             default 0.004   width of the floor patch each sensor looks at (m)
    image_footprint    default 0.0801  width of floor covered by the camera image (m)
    line_threshold     default 0.5     a sensor "sees the line" when its reading is below this
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray


class LineSensorNode(Node):
    def __init__(self):
        super().__init__('line_sensor_node')

        self.n = int(self.declare_parameter('n_sensors', 8).value)
        self.spacing = float(self.declare_parameter('spacing', 0.0095).value)
        self.window = float(self.declare_parameter('window', 0.004).value)
        self.footprint = float(self.declare_parameter('image_footprint', 0.0801).value)
        self.threshold = float(self.declare_parameter('line_threshold', 0.5).value)

        # y position of each sensor, + = left of the robot centre line
        self.y = np.array([((self.n - 1) / 2.0 - i) * self.spacing for i in range(self.n)])

        # Brightness of white floor in the rendered image (learned while running,
        # so it works with any lighting).
        self.white = 0.5

        self.create_subscription(Image, '/line_camera/image', self.on_image, 10)
        self.sensors_pub = self.create_publisher(Float32MultiArray, '/line_sensors', 10)
        self.pos_pub = self.create_publisher(Float32, '/line_position', 10)
        self.get_logger().info(
            f'{self.n} sensors, spacing {self.spacing * 1000:.1f} mm, waiting for /line_camera/image')

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding {msg.encoding}', once=True)
            return

        w, h = msg.width, msg.height
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w * 3]
        gray = data.reshape(h, w, 3).mean(axis=2).mean(axis=0) / 255.0   # one brightness per column

        # Average the columns that fall inside each sensor's window.
        m_per_px = self.footprint / w
        half_px = max(1, int(round(self.window / 2.0 / m_per_px)))
        values = np.zeros(self.n)
        for i, y in enumerate(self.y):
            # image left = robot left = +y
            col = int(round(w / 2.0 - y / m_per_px))
            lo, hi = max(0, col - half_px), min(w, col + half_px + 1)
            values[i] = gray[lo:hi].mean() if hi > lo else 1.0

        self.white = max(self.white, float(values.max()))
        refl = np.clip(values / self.white, 0.0, 1.0)       # 1 = white, 0 = black

        arr = Float32MultiArray()
        arr.data = [float(v) for v in refl]
        self.sensors_pub.publish(arr)

        dark = 1.0 - refl
        dark[refl >= self.threshold] = 0.0                  # only sensors that see the line
        pos = Float32()
        if dark.sum() > 1e-6:
            pos.data = float((dark * self.y).sum() / dark.sum())
        else:
            pos.data = float('nan')                         # line lost
        self.pos_pub.publish(pos)


def main():
    rclpy.init()
    node = LineSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
