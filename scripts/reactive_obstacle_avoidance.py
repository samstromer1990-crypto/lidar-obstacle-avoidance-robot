#!/usr/bin/env python3
"""
Reactive Obstacle Avoidance for Differential-Drive Robot (ROS 2 Jazzy & Gazebo Harmonic)
Subscribes to:
  - /scan (sensor_msgs/msg/LaserScan)
  - /odom (nav_msgs/msg/Odometry)
Publishes to:
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /path (nav_msgs/msg/Path)
"""

import math
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path


class State(Enum):
    FORWARD = "FORWARD"
    STOP_AND_EVALUATE = "STOP_AND_EVALUATE"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    BLOCKED_STOP = "BLOCKED_STOP"


class ReactiveObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_obstacle_avoidance')

        # Configurable Parameters
        self.declare_parameter('forward_speed', 0.22)          # m/s
        self.declare_parameter('turn_speed', 0.5)              # rad/s
        self.declare_parameter('safety_distance', 0.70)        # m (trigger stop)
        self.declare_parameter('clear_distance', 0.95)         # m (resumption threshold)
        self.declare_parameter('min_side_clearance', 0.35)     # m (blocked side limit)
        self.declare_parameter('min_turn_time', 0.8)           # seconds (anti-oscillation)
        self.declare_parameter('scan_timeout', 1.0)            # seconds (fail-safe)

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.clear_dist = self.get_parameter('clear_distance').value
        self.min_side_dist = self.get_parameter('min_side_clearance').value
        self.min_turn_time = self.get_parameter('min_turn_time').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # FSM State & Timing
        self.state = State.FORWARD
        self.state_start_time = self.get_clock().now()
        self.last_scan_time = None

        # Filtered Sector Clearances (meters)
        self.dist_front = math.inf
        self.dist_front_left = math.inf
        self.dist_front_right = math.inf
        self.dist_left = math.inf
        self.dist_right = math.inf

        # Path History for RViz
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"
        self.last_path_pos = None

        # ROS 2 Interfaces
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.on_scan,
            qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # 20 Hz Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Obstacle Avoidance Node Started (ROS 2 Jazzy)')

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        if self.last_path_pos is None or math.hypot(pos.x - self.last_path_pos[0], pos.y - self.last_path_pos[1]) > 0.05:
            self.last_path_pos = (pos.x, pos.y)
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses.pop(0)
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path_msg)

    def on_scan(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()

        ranges_front = []
        ranges_front_left = []
        ranges_front_right = []
        ranges_left = []
        ranges_right = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)

            if -0.26 <= angle <= 0.26:
                ranges_front.append(r)
            elif 0.26 < angle <= 0.70:
                ranges_front_left.append(r)
            elif -0.70 <= angle < -0.26:
                ranges_front_right.append(r)
            elif 0.70 < angle <= 1.57:
                ranges_left.append(r)
            elif -1.57 <= angle < -0.70:
                ranges_right.append(r)

        self.dist_front = min(ranges_front) if ranges_front else msg.range_max
        self.dist_front_left = min(ranges_front_left) if ranges_front_left else msg.range_max
        self.dist_front_right = min(ranges_front_right) if ranges_front_right else msg.range_max
        self.dist_left = min(ranges_left) if ranges_left else msg.range_max
        self.dist_right = min(ranges_right) if ranges_right else msg.range_max

    def transition_to(self, new_state: State):
        if self.state != new_state:
            self.state = new_state
            self.state_start_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()

        if self.last_scan_time is None:
            self.cmd_pub.publish(cmd)
            return

        scan_age = (now - self.last_scan_time).nanoseconds / 1e9
        if scan_age > self.scan_timeout:
            self.get_logger().warn('LiDAR scan timeout! Safety stop.', throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        time_in_state = (now - self.state_start_time).nanoseconds / 1e9
        front_obstacle_dist = min(self.dist_front, self.dist_front_left, self.dist_front_right)

        # State Machine Logic
        if self.state == State.FORWARD:
            if front_obstacle_dist <= self.safety_dist:
                self.get_logger().info(f'Obstacle detected! Front: {front_obstacle_dist:.2f} m')
                self.transition_to(State.STOP_AND_EVALUATE)
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0

        elif self.state == State.STOP_AND_EVALUATE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            left_space = self.dist_left + self.dist_front_left
            right_space = self.dist_right + self.dist_front_right

            if self.dist_left < self.min_side_dist and self.dist_right < self.min_side_dist:
                self.get_logger().warn('Blocked on both sides!')
                self.transition_to(State.BLOCKED_STOP)
            elif left_space >= right_space:
                self.get_logger().info(f'Left: {left_space:.2f} m  Right: {right_space:.2f} m -> Turning LEFT')
                self.transition_to(State.TURN_LEFT)
            else:
                self.get_logger().info(f'Left: {left_space:.2f} m  Right: {right_space:.2f} m -> Turning RIGHT')
                self.transition_to(State.TURN_RIGHT)

        elif self.state == State.TURN_LEFT:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed

            if time_in_state > self.min_turn_time and front_obstacle_dist > self.clear_dist:
                self.get_logger().info('Path clear. Resuming FORWARD.')
                self.transition_to(State.FORWARD)

        elif self.state == State.TURN_RIGHT:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed

            if time_in_state > self.min_turn_time and front_obstacle_dist > self.clear_dist:
                self.get_logger().info('Path clear. Resuming FORWARD.')
                self.transition_to(State.FORWARD)

        elif self.state == State.BLOCKED_STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.dist_left >= self.min_side_dist or self.dist_right >= self.min_side_dist:
                self.transition_to(State.STOP_AND_EVALUATE)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
