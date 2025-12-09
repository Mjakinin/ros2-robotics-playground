#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoTurtle(Node):
    def __init__(self):
        super().__init__('auto_turtle')

        # Publisher für Geschwindigkeiten
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Timer: alle 0.1s aktuellen Befehl publizieren
        self.timer = self.create_timer(0.1, self.control_loop)

        # Aktueller Bewegungsbefehl
        self.current_twist = Twist()

        # Alle X Sekunden neues „Verhalten“ wählen
        self.steps_until_change = 0
        self.min_steps = 10    # 10 * 0.1s = 1s
        self.max_steps = 40    # 40 * 0.1s = 4s

        self.choose_new_behavior()

    def choose_new_behavior(self):
        """Wählt zufällig: vorwärts, links drehen, rechts drehen, langsam kriechen."""
        behavior = random.choice(['forward', 'turn_left', 'turn_right', 'slow_forward'])

        twist = Twist()

        if behavior == 'forward':
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            self.get_logger().info('Verhalten: Geradeaus')
        elif behavior == 'slow_forward':
            twist.linear.x = 1.0
            twist.angular.z = 0.5
            self.get_logger().info('Verhalten: Langsam + leichte Kurve')
        elif behavior == 'turn_left':
            twist.linear.x = 0.0
            twist.angular.z = 2.5
            self.get_logger().info('Verhalten: Links drehen')
        elif behavior == 'turn_right':
            twist.linear.x = 0.0
            twist.angular.z = -2.5
            self.get_logger().info('Verhalten: Rechts drehen')

        self.current_twist = twist
        # Wie lange dieses Verhalten gehalten wird
        self.steps_until_change = random.randint(self.min_steps, self.max_steps)

    def control_loop(self):
        # Aktuellen Befehl senden
        self.cmd_pub.publish(self.current_twist)

        # Countdown runterzählen
        self.steps_until_change -= 1
        if self.steps_until_change <= 0:
            self.choose_new_behavior()


def main(args=None):
    rclpy.init(args=args)
    node = AutoTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
