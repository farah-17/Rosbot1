#!/usr/bin/env python3
import sys
import os
import yaml
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class CourseworkNavClient(Node):
    def __init__(self, goals_file):
        super().__init__('coursework_nav_client')
        self.goals_file = goals_file
        self.goals = self.load_goals()
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_goal_handle = None
        self.navigation_busy = False
        self.lock = threading.Lock()

    def load_goals(self):
        if not os.path.exists(self.goals_file):
            self.get_logger().error(f"Goal file not found: {self.goals_file}")
            return {}
        try:
            with open(self.goals_file, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('goals', {})
        except Exception as e:
            self.get_logger().error(f"Failed to parse YAML file: {e}")
            return {}

    def send_navigation_goal(self, key):
        if key not in self.goals:
            print(f"\n [ERROR] Invalid goal key '{key}'. Please select from the menu.")
            return

        target = self.goals[key]
        x, y = target['x'], target['y']
        name = target.get('name', key)

        print(f"\n [*] Checking navigation action server connection...")
        if not self.client.wait_for_server(timeout_sec=2.0):
            print("\n [ERROR] Navigator server not online! Ensure 'sim.launch.py' or 'navigate.launch.py' is running.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        print(f"\n [->] Sending Goal: '{name}' at position (x={x:.2f}, y={y:.2f})...")
        with self.lock:
            self.navigation_busy = True

        future = self.client.send_goal_async(goal_msg, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("\n [REJECTED] Goal was REJECTED by navigator (target in a wall or outside map boundary).")
            with self.lock:
                self.navigation_busy = False
            return

        self.active_goal_handle = goal_handle
        print(" [ACCEPTED] Goal ACCEPTED by navigator. Robot driving...")
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist_rem = fb.distance_remaining
        time_elapsed = fb.navigation_time.sec
        est_rem = dist_rem / 0.15 if 0.15 > 0 else 0
        print(f"\r [DRIVING] {dist_rem:.2f} m to go | Est. time remaining: {est_rem:.0f}s (Elapsed: {time_elapsed}s)   ", end="", flush=True)

    def on_result(self, future):
        result_status = future.result().status
        print("\n")
        if result_status == GoalStatus.STATUS_SUCCEEDED:
            print(" [SUCCESS] Goal COMPLETED: Robot arrived safely at destination!")
        elif result_status == GoalStatus.STATUS_CANCELED:
            print(" [CANCELED] Goal CANCELLED by user/system request.")
        elif result_status == GoalStatus.STATUS_ABORTED:
            print(" [FAILED] Goal ABORTED: Path blocked or timeout reached.")
        else:
            print(f" [FAILED] Goal finished with status code: {result_status}")
        with self.lock:
            self.navigation_busy = False
            self.active_goal_handle = None

    def cancel_current_goal(self):
        if self.active_goal_handle:
            print("\n [!] Canceling current goal execution...")
            self.active_goal_handle.cancel_goal_async()

def main():
    rclpy.init()
    yaml_path = '/root/raspbot_lidar_ws/src/raspbot_navigation/config/goals.yaml'
    node = CourseworkNavClient(yaml_path)
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            print("\n" + "="*50)
            print("          RASPBOT NAVIGATION MENU")
            print("="*50)
            keys = list(node.goals.keys())
            if not keys:
                print(" [ERROR] No goals loaded from configuration file!")
                break
            for idx, key in enumerate(keys, 1):
                g = node.goals[key]
                print(f"  {idx}. {key:<10} - {g.get('name', 'N/A')} (x: {g['x']}, y: {g['y']})")
            print("  c. Cancel Active Goal")
            print("  q. Quit Application")
            print("="*50)

            choice = input("Select destination choice: ").strip().lower()
            if choice == 'q':
                print("\nExiting navigation application...")
                break
            elif choice == 'c':
                node.cancel_current_goal()
                time.sleep(1.0)
            elif choice.isdigit() and 1 <= int(choice) <= len(keys):
                selected_key = keys[int(choice) - 1]
                node.send_navigation_goal(selected_key)
                while node.navigation_busy:
                    time.sleep(0.2)
            else:
                print("\n [WARNING] Invalid selection. Please enter a valid number or option.")
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n [!] Keyboard interrupt detected (Ctrl+C). Shutting down node cleanly...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
