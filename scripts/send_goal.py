import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main():
    rclpy.init()
    node = Node('test_goal_sender')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    print("Waiting for /navigate_to_pose action server...")
    if not client.wait_for_server(timeout_sec=5.0):
        print("Server not available!")
        return

    x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp.sec = 0
    goal.pose.header.stamp.nanosec = 0
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0

    print(f"Sending goal to ({x}, {y})...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        print("Goal rejected by server!")
        return

    print("Goal accepted! Monitoring result...")
    res_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    result = res_future.result()
    print(f"Goal completed with status: {result.status}, error_code: {result.result.error_code}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
