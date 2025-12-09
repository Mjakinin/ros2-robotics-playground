# ROS2 Robotics Playground

Experiments with ROS 2 (Kilted, Ubuntu 24.04, WSL) for learning robotics.

## Packages

### `auto_turtle`
- Autonomous controller for `turtlesim`
- Subscribes to `/turtle1/pose`
- Publishes velocity commands to `/turtle1/cmd_vel`

### `tb3_obstacle_avoid`
- Obstacle avoidance node for a simulated TurtleBot3-like robot
- Subscribes to `sensor_msgs/LaserScan` on `/scan`
- Publishes `geometry_msgs/TwistStamped` on `/cmd_vel`
- Implements simple logic:
  - drive forward if `min_front > safe_distance`
  - rotate or back up if obstacle is too close

### `tb3_fake_scan`
- Simple fake LiDAR publisher for `/scan`
- Allows testing `tb3_obstacle_avoid` even when the sim doesn’t publish real scans

## Status

Tested with:
- ROS 2 Kilted (Ubuntu 24.04, WSL)
- TurtleBot3 Gazebo simulation (Kilted debian packages)

Known limitations:
- On this setup the Gazebo/ros_gz_bridge stack sometimes does not move the robot
  even though `/cmd_vel` and `/scan` are correct. The controller nodes, however,
  can be reused on stable distributions (e.g. Humble).
