"""ROS 2 node: plan a path with A* on the saved map and drive the robot along it.

It offers the same action as Nav2 on a TurtleBot, so any Nav2 client works:
    action  /navigate_to_pose  (nav2_msgs/action/NavigateToPose)
also accepts RViz's "2D Goal Pose" button:
    topic   /goal_pose         (geometry_msgs/PoseStamped)
and publishes:
    /plan               (nav_msgs/Path)       the planned path, for RViz and the GUI
    /cmd_vel            (geometry_msgs/Twist) speed commands for motor_driver
    /navigation_status  (std_msgs/String)     what the navigator is doing, in words
It reads the robot's position from TF (map -> base_link), which comes from
Cartographer on the real robot, or from sim_robot in the simulator.

Live obstacles: the saved map is never changed, but every lidar scan is also
turned into temporary obstacles (live_layer.py). If one appears on the path,
or blocks the way ahead, the robot stops and plans a new path around it. It
tries up to `max_replans` times before giving up. The number of replans is
reported in the action feedback (number_of_recoveries).
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from raspbot_navigation.astar import astar, path_length, simplify
from raspbot_navigation.grid_map import GridMap
from raspbot_navigation.live_layer import LiveObstacleLayer


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Angle in -pi..pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('map_yaml', '/root/raspbot_lidar_ws/maps/my_room.yaml')
        self.declare_parameter('inflation_radius', 0.20)   # m kept free around walls
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('linear_speed', 0.15)       # m/s when driving straight
        self.declare_parameter('angular_speed', 0.6)       # rad/s when turning on the spot
        self.declare_parameter('heading_tolerance', 0.35)  # rad: turn on the spot above this
        self.declare_parameter('waypoint_tolerance', 0.12)  # m: waypoint reached
        self.declare_parameter('goal_tolerance', 0.10)     # m: goal reached
        self.declare_parameter('obstacle_distance', 0.20)  # m: stop if something is closer ahead
        self.declare_parameter('blocked_timeout', 8.0)     # s blocked before giving up
        self.declare_parameter('max_time', 180.0)          # s for one goal before giving up
        # Live obstacles and replanning
        self.declare_parameter('live_obstacles', True)     # False = use only the saved map
        self.declare_parameter('live_min_range', 0.12)     # m: ignore lidar points closer than this
        self.declare_parameter('live_max_range', 2.0)      # m: ignore lidar points further than this
        self.declare_parameter('live_ttl', 3.0)            # s an obstacle stays after its last hit
        self.declare_parameter('live_min_hits', 2)         # scans a cell must be hit in
        self.declare_parameter('live_inflation', 0.15)     # m kept free around a live obstacle
        self.declare_parameter('max_replans', 5)           # new paths to try before giving up
        self.declare_parameter('replan_blocked_after', 1.5)  # s blocked ahead before replanning
        self.declare_parameter('replan_cooldown', 2.0)     # s between two replans

        self.grid = GridMap(self.p('map_yaml'))
        self.grid.inflate(self.p('inflation_radius'))
        self.get_logger().info(
            f'map {self.p("map_yaml")}: {self.grid.width} x {self.grid.height} cells, '
            f'{self.grid.resolution} m, inflated by {self.p("inflation_radius")} m')
        self.live = None
        if self.p('live_obstacles'):
            self.live = LiveObstacleLayer(
                self.grid, min_range=self.p('live_min_range'),
                max_range=self.p('live_max_range'), ttl=self.p('live_ttl'),
                min_hits=self.p('live_min_hits'), inflation=self.p('live_inflation'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        group = ReentrantCallbackGroup()   # lets callbacks run while a goal is driving
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data,
                                 callback_group=group)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal_pose, 10,
                                 callback_group=group)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=self.on_goal_request,
                                   handle_accepted_callback=self.on_accepted,
                                   cancel_callback=lambda goal: CancelResponse.ACCEPT,
                                   callback_group=group)
        self.self_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                        callback_group=group)
        self.lock = threading.Lock()
        self.active_goal = None
        self.front_distance = math.inf
        self.scan_count = 0
        self.get_logger().info('navigator ready: send a goal on /navigate_to_pose')

    def p(self, name):
        return self.get_parameter(name).value

    def say(self, text):
        """Log a message and also publish it, so the terminal menu can show it."""
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    # ---- where is the robot? ---------------------------------------------
    def robot_pose(self):
        """(x, y, yaw) of the robot in the map, or None if TF has no answer."""
        try:
            t = self.tf_buffer.lookup_transform(self.p('global_frame'), self.p('robot_frame'),
                                                Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quaternion(t.transform.rotation)

    # ---- the lidar: is something in front? and what is around us? ---------
    def on_scan(self, msg):
        laser = (0.0, 0.0, 0.0)
        try:   # the lidar may be mounted turned round: find its pose on the robot
            t = self.tf_buffer.lookup_transform(self.p('robot_frame'), msg.header.frame_id, Time())
            laser = (t.transform.translation.x, t.transform.translation.y,
                     yaw_from_quaternion(t.transform.rotation))
        except TransformException:
            pass
        closest = math.inf
        low = max(msg.range_min, 0.10)
        for i in range(0, len(msg.ranges), 2):       # every 2nd point is enough (saves CPU)
            r = msg.ranges[i]
            if not (low < r < msg.range_max):
                continue
            angle = wrap(msg.angle_min + i * msg.angle_increment + laser[2])
            if abs(angle) < math.radians(25):
                closest = min(closest, r)
        self.front_distance = closest

        self.scan_count += 1
        if self.live is not None and self.scan_count % 2 == 0:   # live layer: 5 scans a second
            pose = self.robot_pose()
            if pose is not None:
                self.live.update(time.monotonic(), msg.ranges, msg.angle_min,
                                 msg.angle_increment, msg.range_min, msg.range_max,
                                 laser, pose)

    # ---- goals --------------------------------------------------------------
    def on_goal_pose(self, msg):
        """RViz '2D Goal Pose' -> send it to our own action server."""
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.self_client.send_goal_async(goal)

    def on_goal_request(self, goal_request):
        """Accept or reject a new goal straight away (before driving)."""
        pose = goal_request.pose.pose.position
        frame = goal_request.pose.header.frame_id or self.p('global_frame')
        if frame != self.p('global_frame'):
            self.get_logger().warn(f'Rejected: goal is in frame "{frame}", not "map"')
            return GoalResponse.REJECT
        col, row = self.grid.world_to_cell(pose.x, pose.y)
        if not self.grid.inside(col, row):
            self.get_logger().warn(f'Rejected: ({pose.x:.2f}, {pose.y:.2f}) is outside the map')
            return GoalResponse.REJECT
        # Only the saved map decides here: a person standing on the goal is not a wall.
        if self.grid.nearest_free(col, row, 4, live=False) is None:
            self.get_logger().warn(f'Rejected: ({pose.x:.2f}, {pose.y:.2f}) is in a wall')
            return GoalResponse.REJECT
        self.get_logger().info(f'Goal accepted: ({pose.x:.2f}, {pose.y:.2f})')
        return GoalResponse.ACCEPT

    def on_accepted(self, goal_handle):
        with self.lock:      # a new goal replaces the old one
            self.active_goal = goal_handle
        goal_handle.execute()

    # ---- planning ---------------------------------------------------------------
    def refresh_live(self, xy):
        """Copy the lidar's current obstacles into the grid. Returns how many cells."""
        if self.live is None:
            return 0
        return self.live.apply(time.monotonic(), xy)

    def plan(self, start_xy, goal_xy):
        """A* from start to goal (metres), around live obstacles too.
        Returns a list of (x, y) waypoints or None."""
        live_cells = self.refresh_live(start_xy)
        start = self.grid.nearest_free(*self.grid.world_to_cell(*start_xy), 6)
        goal = self.grid.nearest_free(*self.grid.world_to_cell(*goal_xy), 4)
        if start is None or goal is None:
            return None
        cells, expanded = astar(self.grid, start, goal)
        if cells is None:
            self.get_logger().warn(
                f'A*: no path (expanded {expanded} cells, {live_cells} live obstacle cells)')
            return None
        waypoints = simplify(self.grid, cells)
        self.get_logger().info(
            f'A*: {len(cells)} cells, {path_length(self.grid, cells):.2f} m, '
            f'{len(waypoints)} waypoints, expanded {expanded} cells, '
            f'{live_cells} live obstacle cells')
        points = [self.grid.cell_to_world(c, r) for c, r in waypoints]
        if self.grid.is_free(*self.grid.world_to_cell(*goal_xy)):
            points[-1] = goal_xy             # end exactly on the clicked point
        return points

    def live_blocks_path(self, xy, points, index):
        """True if a live obstacle now lies on the rest of the planned path."""
        if not self.grid.dynamic:
            return False
        chain = [xy] + points[index:]
        cells = [self.grid.world_to_cell(*p) for p in chain]
        return any(self.grid.live_on_line(a, b) for a, b in zip(cells, cells[1:]))

    def on_map(self, x, y):
        """True if (x, y) is on the free part of the saved map (or within 0.3 m of it).
        False means localisation is lost: the robot thinks it is outside the room."""
        col, row = self.grid.world_to_cell(x, y)
        return (self.grid.inside(col, row)
                and self.grid.nearest_free(col, row, 10, live=False) is not None)

    def publish_plan(self, points):
        msg = Path()
        msg.header.frame_id = self.p('global_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.plan_pub.publish(msg)

    def drive(self, v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    # ---- driving one goal -------------------------------------------------
    def execute(self, goal_handle):
        target = goal_handle.request.pose.pose.position
        goal_xy = (target.x, target.y)
        result = NavigateToPose.Result()

        def finish(outcome, text):
            self.drive(0.0, 0.0)
            self.say(text)           # published before the result, so the menu can show why
            if outcome == 'succeed':
                goal_handle.succeed()
            elif outcome == 'cancel':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        pose = None
        for _ in range(20):                  # wait up to 2 s for a position
            pose = self.robot_pose()
            if pose:
                break
            time.sleep(0.1)
        if pose is None:
            return finish('abort', 'Failed: robot position unknown (no TF map -> base_link)')

        lost = ('Failed: the robot is outside the map (localisation lost). Put it on its '
                'start spot and set the start position again')
        if not self.on_map(*pose[:2]):
            return finish('abort', lost)

        points = self.plan(pose[:2], goal_xy)
        if points is None:
            self.publish_plan([])
            return finish('abort', 'Failed: no path to the goal')
        self.publish_plan(points)

        started = time.monotonic()
        blocked_since = None
        next_check = 0.0          # when to look for obstacles on the path again
        next_replan = 0.0         # no new replan before this time
        replans = 0
        index = 1 if len(points) > 1 else 0
        while rclpy.ok():
            if self.active_goal is not goal_handle:
                return finish('abort', 'Stopped: replaced by a newer goal')
            if goal_handle.is_cancel_requested:
                return finish('cancel', 'Cancelled')
            now = time.monotonic()
            if now - started > self.p('max_time'):
                return finish('abort', 'Failed: took too long')

            pose = self.robot_pose()
            if pose is None:
                self.drive(0.0, 0.0)
                time.sleep(0.1)
                continue
            x, y, yaw = pose
            if not self.on_map(x, y):        # the position estimate jumped out of the room
                return finish('abort', lost)
            wx, wy = points[index]
            distance = math.hypot(wx - x, wy - y)
            last = index == len(points) - 1

            if last and distance < self.p('goal_tolerance'):
                return finish('succeed', 'Goal reached')
            if not last and distance < self.p('waypoint_tolerance'):
                index += 1
                continue

            # Is there a reason to plan a new path? (a new obstacle on it, or blocked ahead)
            reason = None
            if self.live is not None and now >= next_check:
                next_check = now + 0.5
                self.refresh_live((x, y))
                if self.live_blocks_path((x, y), points, index):
                    reason = 'new obstacle on the path'
            if (blocked_since is not None and self.live is not None
                    and now - blocked_since > self.p('replan_blocked_after')):
                reason = 'obstacle ahead'
            if reason and now >= next_replan:
                replans += 1
                next_replan = now + self.p('replan_cooldown')
                if replans > self.p('max_replans'):
                    return finish('abort', 'Failed: path blocked by an obstacle, no way '
                                           f'around it after {self.p("max_replans")} replans')
                self.drive(0.0, 0.0)
                self.say(f'Replanning ({replans}/{self.p("max_replans")}): {reason}')
                new_points = self.plan((x, y), goal_xy)
                if new_points is None:
                    self.say('No way around the obstacle yet, waiting for it to clear')
                else:
                    points = new_points
                    index = 1 if len(points) > 1 else 0
                    blocked_since = None
                    self.publish_plan(points)
                continue

            error = wrap(math.atan2(wy - y, wx - x) - yaw)
            if abs(error) > self.p('heading_tolerance'):
                # 1. Facing the wrong way: turn on the spot
                self.drive(0.0, math.copysign(self.p('angular_speed'), error))
                blocked_since = None
            elif self.front_distance < self.p('obstacle_distance'):
                # 2. Something in the way: wait (and replan), give up after a while
                self.drive(0.0, 0.0)
                blocked_since = blocked_since or time.monotonic()
                if time.monotonic() - blocked_since > self.p('blocked_timeout'):
                    return finish('abort', 'Failed: path blocked by an obstacle')
            else:
                # 3. Roughly facing the waypoint: drive and steer a little
                blocked_since = None
                w = max(-self.p('angular_speed'), min(self.p('angular_speed'), 1.5 * error))
                self.drive(self.p('linear_speed'), w)

            feedback = NavigateToPose.Feedback()
            feedback.current_pose.header.frame_id = self.p('global_frame')
            feedback.current_pose.pose.position.x = x
            feedback.current_pose.pose.position.y = y
            feedback.number_of_recoveries = replans
            feedback.distance_remaining = float(distance + sum(
                math.dist(a, b) for a, b in zip(points[index:], points[index + 1:])))
            elapsed = time.monotonic() - started
            feedback.navigation_time.sec = int(elapsed)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)                  # 10 times a second
        return finish('abort', 'Stopped: shutting down')

    def destroy_node(self):
        self.drive(0.0, 0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = Navigator()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
