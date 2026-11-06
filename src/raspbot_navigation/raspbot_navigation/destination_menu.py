#!/usr/bin/env python3
"""Terminal destination menu for the Raspbot navigator.

Pick a destination from a list; the robot drives there by itself using the
navigator node (A* + turn-then-drive) through the standard action
/navigate_to_pose. While it drives, one status line is updated live with:

    * the navigator's feedback (distance still to go, elapsed time)
    * the robot's position and heading (TF map -> base_link, from Cartographer)
    * live lidar distances (closest point in front, closest point anywhere)

Destinations are stored in a YAML file, not in the code:

    destinations:
      - {name: Home, x: 0.0, y: 0.0}
      - {name: Laboratory, x: 2.5, y: 1.0}

Run (inside the container, with navigate.launch.py already running):

    python3 destination_menu.py
    python3 destination_menu.py --ros-args -p destinations_file:=/path/to/file.yaml

While the robot drives: type  c  + Enter (or press Ctrl+C) to cancel.
"""
import math
import os
import select
import sys
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULT_FILE = '/root/raspbot_lidar_ws/src/raspbot_navigation/config/destinations.yaml'
DEFAULT_DESTINATIONS = [
    {'name': 'Home', 'x': -0.01, 'y': -0.01},
    {'name': 'Blue Box', 'x': 0.23, 'y': 0.44},
    {'name': 'Small box', 'x': 0.32, 'y': 1.19},
    {'name': 'White box', 'x': 0.83, 'y': 0.59},
    {'name': 'The book', 'x': 0.32,  'y': 0.74},
]
FRONT_HALF_ANGLE = math.radians(25)   # same cone the navigator uses
WARN_DISTANCE = 0.30                  # m: show an OBSTACLE warning below this
SCAN_TIMEOUT = 1.5                    # s without /scan -> "no lidar data"


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def metres(d):
    return '--' if d is None or math.isinf(d) else f'{d:.2f} m'


class MenuClient(Node):
    """ROS side: action client, TF, lidar and the /initialpose publisher."""

    def __init__(self):
        super().__init__('destination_menu')
        self.declare_parameter('destinations_file', DEFAULT_FILE)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # What the navigator says it is doing ("Replanning ...", "Failed: ...")
        self.create_subscription(String, 'navigation_status', self.on_status, 10)

        self.lock = threading.Lock()
        self.front = math.inf       # closest lidar point in the front cone
        self.nearest = math.inf     # closest lidar point in any direction
        self.last_scan = 0.0        # monotonic time of the last scan

        self.goal_handle = None
        self.reset_goal_state()

    def p(self, name):
        return self.get_parameter(name).value

    def reset_goal_state(self):
        self.state = 'idle'         # idle / sending / driving / done
        self.distance = None
        self.nav_time = 0
        self.outcome = None         # text for the user once finished
        self.replans = 0            # how many times the navigator planned a new path
        self.events = []            # navigator messages not shown yet
        self.last_status = None     # the navigator's latest message (the reason if it failed)
        self.finished = threading.Event()
        self.answered = threading.Event()
        self.accepted = False

    # ---- lidar -----------------------------------------------------------
    def on_scan(self, msg):
        try:  # the lidar is mounted turned round: use its yaw from TF
            t = self.tf_buffer.lookup_transform(self.p('robot_frame'), msg.header.frame_id, Time())
            laser_yaw = yaw_from_quaternion(t.transform.rotation)
        except TransformException:
            laser_yaw = 0.0
        low = max(msg.range_min, 0.10)
        front = nearest = math.inf
        for i, r in enumerate(msg.ranges):
            if not (low < r < msg.range_max):   # also skips nan / inf
                continue
            nearest = min(nearest, r)
            angle = wrap(msg.angle_min + i * msg.angle_increment + laser_yaw)
            if abs(angle) < FRONT_HALF_ANGLE:
                front = min(front, r)
        with self.lock:
            self.front, self.nearest = front, nearest
            self.last_scan = time.monotonic()

    def lidar_text(self):
        with self.lock:
            age = time.monotonic() - self.last_scan if self.last_scan else None
            front, nearest = self.front, self.nearest
        if age is None or age > SCAN_TIMEOUT:
            return 'lidar: NO DATA'
        warn = '  !! OBSTACLE AHEAD' if front < WARN_DISTANCE else ''
        return f'lidar: front {metres(front)}, nearest {metres(nearest)}{warn}'

    # ---- robot position ------------------------------------------------
    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.p('global_frame'), self.p('robot_frame'),
                                                Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    def pose_text(self):
        pose = self.robot_pose()
        if pose is None:
            return 'robot: position unknown'
        return f'robot: x={pose[0]:.2f} y={pose[1]:.2f} hdg={math.degrees(pose[2]):.0f}deg'

    # ---- start position (replaces the map window's "Start") ----------------
    def send_start(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p('global_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.initial_pose_pub.publish(msg)

    # ---- the action: send goal -> accepted? -> feedback -> result -----------
    def send_goal(self, x, y):
        """Send a goal. Returns (ok, message). Blocks until accepted/rejected."""
        if not self.client.wait_for_server(timeout_sec=2.0):
            return False, 'Navigator is not running (no /navigate_to_pose server).'
        self.reset_goal_state()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.p('global_frame')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.w = 1.0
        self.state = 'sending'
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        if not self.answered.wait(timeout=5.0):
            self.state = 'idle'
            return False, 'No answer from the navigator (timeout).'
        if not self.accepted:
            self.state = 'idle'
            return False, 'Goal REJECTED (outside the map or inside a wall).'
        self.state = 'driving'
        return True, 'Goal ACCEPTED.'

    def on_goal_response(self, future):
        handle = future.result()
        self.accepted = bool(handle.accepted)
        if self.accepted:
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_result)
        self.answered.set()

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.distance = fb.distance_remaining
        self.nav_time = fb.navigation_time.sec
        self.replans = fb.number_of_recoveries

    def on_status(self, msg):
        with self.lock:
            self.last_status = msg.data
            # "Goal reached" etc. are reported through the result; show only progress events
            if msg.data.startswith(('Replanning', 'No way around')):
                self.events.append(msg.data)

    def on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.outcome = 'Goal COMPLETED: destination reached.'
        elif status == GoalStatus.STATUS_CANCELED:
            self.outcome = 'Goal CANCELLED.'
        else:
            reason = self.last_status or 'see the navigator terminal for the reason'
            self.outcome = f'Goal FAILED. {reason}'
        self.goal_handle = None
        self.state = 'done'
        self.finished.set()

    def cancel(self):
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        self.cmd_pub.publish(Twist())   # and stop the wheels straight away

    # ---- one navigation, with a live status line ------------------------------
    def status_line(self, name):
        dist = f'{self.distance:.2f} m to go' if self.distance is not None else 'planning...'
        replans = f', replans {self.replans}' if self.replans else ''
        return (f'-> {name}: {dist}, {self.nav_time}s{replans} | {self.pose_text()} | '
                f'{self.lidar_text()}')

    def drive_to(self, name, x, y):
        ok, message = self.send_goal(x, y)
        print(message)
        if not ok:
            return
        print("Driving. Type 'c' + Enter (or Ctrl+C) to cancel.")
        try:
            while not self.finished.wait(timeout=0.25):
                with self.lock:
                    events, self.events = self.events, []
                for event in events:      # e.g. "Replanning (1/5): new obstacle on the path"
                    sys.stdout.write('\r\x1b[K' + event + '\n')
                sys.stdout.write('\r\x1b[K' + self.status_line(name))
                sys.stdout.flush()
                if select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline().strip().lower().startswith('c'):
                        sys.stdout.write('\n')
                        print('Cancelling ...')
                        self.cancel()
        except KeyboardInterrupt:
            sys.stdout.write('\n')
            print('Ctrl+C: cancelling ...')
            self.cancel()
            self.finished.wait(timeout=3.0)
        sys.stdout.write('\r\x1b[K')
        print(self.outcome or 'Navigation ended (no result received).')
        pose = self.robot_pose()
        if pose is not None and self.outcome and 'COMPLETED' in self.outcome:
            err = math.hypot(pose[0] - x, pose[1] - y) * 100.0
            print(f'Final position x={pose[0]:.2f} y={pose[1]:.2f} m, '
                  f'{err:.0f} cm from the destination.')


# ---- destinations file ---------------------------------------------------------
def load_destinations(path):
    """Return a list of {'name', 'x', 'y'}. Bad entries are skipped with a message."""
    if not os.path.isfile(path):
        print(f'(No destinations file at {path}; using built-in defaults.)')
        return [dict(d) for d in DEFAULT_DESTINATIONS]
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        items = data.get('destinations', [])
    except (OSError, yaml.YAMLError, AttributeError) as err:
        print(f'Could not read {path}: {err}. Using built-in defaults.')
        return [dict(d) for d in DEFAULT_DESTINATIONS]
    result = []
    for n, item in enumerate(items, 1):
        try:
            result.append({'name': str(item['name']), 'x': float(item['x']),
                           'y': float(item['y'])})
        except (KeyError, TypeError, ValueError):
            print(f'Skipping destination #{n}: needs name, x and y.')
    if not result:
        print('No valid destinations in the file; using built-in defaults.')
        return [dict(d) for d in DEFAULT_DESTINATIONS]
    return result


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return '0'


def set_start(ros):
    """Tell Cartographer where the robot is (the map window does this normally)."""
    print('Place the robot, then enter its position in the map frame.')
    raw = ask('x y heading_deg  [Enter = 0 0 0, the start spot]: ')
    try:
        x, y, deg = (float(v) for v in raw.split()) if raw else (0.0, 0.0, 0.0)
    except ValueError:
        print('Invalid input: type three numbers, for example:  0 0 0')
        return
    ros.send_start(x, y, math.radians(deg))
    print(f'Start sent: x={x:.2f} y={y:.2f} heading={deg:.0f} deg. '
          'Check the robot position below before driving.')


def print_menu(ros, destinations):
    print('\n==============================================')
    print('       RASPBOT AUTONOMOUS NAVIGATION')
    print('==============================================')
    for number, d in enumerate(destinations, 1):
        print(f'  {number}. {d["name"]:<18} ({d["x"]:.2f}, {d["y"]:.2f})')
    print()
    print('  S. Set start position (localise)')
    print('  R. Reload destinations')
    print('  0. Exit')
    print('==============================================')
    print(ros.pose_text())
    print(ros.lidar_text())


def main():
    rclpy.init()
    ros = MenuClient()
    threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()
    ros.get_logger().info('Terminal destination navigator initialized.')
    path = ros.p('destinations_file')
    destinations = load_destinations(path)
    time.sleep(0.5)   # let TF and /scan arrive before the first menu

    try:
        while True:
            print_menu(ros, destinations)
            choice = ask('\nSelect a destination: ').lower()
            if choice == '':
                continue
            if choice == '0':
                break
            if choice == 'r':
                destinations = load_destinations(path)
                print(f'Reloaded {len(destinations)} destinations.')
            elif choice == 's':
                set_start(ros)
                time.sleep(1.0)
            elif choice.isdigit() and 1 <= int(choice) <= len(destinations):
                d = destinations[int(choice) - 1]
                print(f'\nGoing to {d["name"]} ({d["x"]:.2f}, {d["y"]:.2f}) ...')
                ros.drive_to(d['name'], d['x'], d['y'])
            else:
                print(f'Invalid choice "{choice}". Type a number from the list, S, R or 0.')
    except KeyboardInterrupt:
        print('\nBye.')
    finally:
        ros.cancel()
        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
