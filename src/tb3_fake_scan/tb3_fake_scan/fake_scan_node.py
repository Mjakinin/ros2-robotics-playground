#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanNode(Node):
    def __init__(self):
        super().__init__('fake_scan_node')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

        # einfache Scan-Parameter
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 0.01
        self.range_min = 0.12
        self.range_max = 3.5

        self.get_logger().info("FakeScanNode gestartet, publiziert /scan")

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_scan"

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # alle Strahlen „frei“ (z.B. 3 m entfernt)
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        msg.ranges = [3.0] * num_readings

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
