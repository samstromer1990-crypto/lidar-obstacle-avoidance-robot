#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop')
        # Distances are measured from the chassis edge, not from the lidar
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('slow_distance', 0.90)
        self.declare_parameter('lidar_x', 0.10)             # lidar position in base_link
        self.declare_parameter('front_edge', 0.20)          # chassis front in base_link
        self.declare_parameter('rear_edge', -0.20)          # chassis rear in base_link
        self.declare_parameter('corridor_half_width', 0.18)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_timeout', 1.0)

        self.min_front = math.inf
        self.min_rear = math.inf
        self.last_scan_time = None
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.sent_idle_stop = True

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_in', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)  # 20 Hz

    def on_scan(self, msg: LaserScan):
        lidar_x = self.get_parameter('lidar_x').value
        front_edge = self.get_parameter('front_edge').value
        rear_edge = self.get_parameter('rear_edge').value
        half_w = self.get_parameter('corridor_half_width').value

        best_front = math.inf
        best_rear = math.inf
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            x = lidar_x + r * math.cos(a)   # point position in base_link
            y = r * math.sin(a)
            if abs(y) > half_w:
                continue                    # not in the robot's path
            if x > front_edge - 0.05:       # ahead of the nose
                best_front = min(best_front, x - front_edge)
            elif x < rear_edge + 0.05:      # behind the tail
                best_rear = min(best_rear, rear_edge - x)
        self.min_front = best_front
        self.min_rear = best_rear
        self.last_scan_time = self.get_clock().now()

    def on_cmd(self, cmd: Twist):
        self.last_cmd = cmd
        self.last_cmd_time = self.get_clock().now()
        self.sent_idle_stop = False

    def tick(self):
        if self.last_cmd_time is None:
            return
        now = self.get_clock().now()
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9

        # No fresh command: stop once, then stay quiet
        if cmd_age > self.get_parameter('cmd_timeout').value:
            if not self.sent_idle_stop:
                self.pub.publish(Twist())
                self.sent_idle_stop = True
            return

        stop = self.get_parameter('stop_distance').value
        slow = self.get_parameter('slow_distance').value
        out = Twist()
        out.linear = self.last_cmd.linear
        out.angular = self.last_cmd.angular
        vx = self.last_cmd.linear.x

        if vx != 0.0:
            scan_age = math.inf
            if self.last_scan_time is not None:
                scan_age = (now - self.last_scan_time).nanoseconds / 1e9

            if scan_age > self.get_parameter('scan_timeout').value:
                out.linear.x = 0.0
                self.get_logger().warn('No recent /scan, blocking linear motion',
                                       throttle_duration_sec=1.0)
            else:
                dist = self.min_front if vx > 0.0 else self.min_rear
                side = 'ahead' if vx > 0.0 else 'behind'
                if dist <= stop:
                    out.linear.x = 0.0
                    self.get_logger().warn(
                        f'Obstacle {dist:.2f} m {side}, blocking motion',
                        throttle_duration_sec=1.0)
                elif dist < slow:
                    out.linear.x = vx * (dist - stop) / (slow - stop)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObstacleStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()