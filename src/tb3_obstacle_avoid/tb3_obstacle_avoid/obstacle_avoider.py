#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Publisher: TwistStamped auf /cmd_vel
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber: LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Parameter
        self.safe_distance = 0.6
        self.forward_speed = 0.15
        self.turn_speed = 0.4  # etwas langsamer, damit er nicht umkippt

        self.current_min_front = float('inf')
        self.get_logger().info("Obstacle Avoider (TwistStamped) gestartet! Subscribed to /scan")

        # 10 Hz Control-Loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        # mittleres Drittel = vorne
        n = len(msg.ranges)
        front_ranges = msg.ranges[n // 3: 2 * n // 3]
        valid = [r for r in front_ranges if math.isfinite(r)]
        self.current_min_front = min(valid) if valid else float('inf')

        # nur ab und zu loggen, damit es nicht zu spammy wird
        self.get_logger().debug(f"min_front = {self.current_min_front:.2f} m")

    def control_loop(self):
        if self.current_min_front == float('inf'):
            # noch keine Daten bekommen
            self.get_logger().warn("Noch keine LaserScan-Daten erhalten...")
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        if self.current_min_front < 0.25:
            cmd.twist.linear.x = -0.05
            cmd.twist.angular.z = 0.3
            self.get_logger().warn(
                f"STUCK! ({self.current_min_front:.2f} m) -> Rückwärts + Drehen"
            )
        elif self.current_min_front < self.safe_distance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.turn_speed
            self.get_logger().info(
                f"Hindernis -> Drehen ({self.current_min_front:.2f} m)"
            )
        else:
            cmd.twist.linear.x = self.forward_speed
            cmd.twist.angular.z = 0.0
            self.get_logger().info(
                f"Frei -> Vorwärts (min_front={self.current_min_front:.2f} m)"
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
