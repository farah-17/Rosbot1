"""A small window to drive the robot by clicking on the saved map.

    ros2 run raspbot_navigation nav_gui --ros-args -p map_yaml:=/root/raspbot_lidar_ws/maps/my_room.yaml

  * "Start" mode: click (and drag to show the direction) where the robot is.
    The GUI publishes /initialpose.
  * "Destination" mode: click where the robot should go. A* draws a preview.
  * Go: sends the destination to the navigator (/navigate_to_pose action) and
    shows whether the goal was accepted, completed or failed (and why).
  * Stop: cancels the goal.

The saved map never changes. On top of it the window draws what the lidar sees
RIGHT NOW: purple dots are lidar points, red squares are live obstacles (boxes,
people, moved furniture). The A* preview avoids them too. If the robot's
position falls outside the room, a red warning says that localisation is lost.
"""

import math
import threading
import time
import tkinter as tk

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from raspbot_navigation.astar import astar, path_length, simplify
from raspbot_navigation.grid_map import FREE, OCCUPIED, GridMap
from raspbot_navigation.live_layer import LiveObstacleLayer, scan_points_in_map
from raspbot_navigation.navigator import yaw_from_quaternion

COLOURS = {'free': '#ffffff', 'wall': '#000000', 'unknown': '#b4b4b4', 'inflated': '#f4c6c6'}
LIDAR_DOT = '#8e24aa'       # purple: one lidar point
LIVE_OBSTACLE = '#e53935'   # red: a live obstacle cell
DRAW_RANGE = 3.0            # m: lidar points further away are not drawn
REFRESH_MS = 250            # redraw 4 times a second (the Pi also runs Cartographer)


class NavClient(Node):
    """The ROS side of the GUI: publishers, the action client, TF and the lidar."""

    def __init__(self):
        super().__init__('nav_gui')
        self.declare_parameter('map_yaml', '/root/raspbot_lidar_ws/maps/my_room.yaml')
        self.declare_parameter('inflation_radius', 0.20)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'navigation_status', self.on_nav_status, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Every action has this cancel service. An empty request cancels ALL goals,
        # also ones sent from a terminal or from RViz.
        self.cancel_all = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.plan = []            # latest path from the navigator, list of (x, y)
        self.status = 'Ready. Choose a start (optional) and a destination.'
        self.goal_handle = None
        self.layer = None         # LiveObstacleLayer, set by the window once the map is loaded
        self.scan_points = []     # latest lidar points in map coordinates, for drawing
        self.scan_version = 0     # grows with every processed scan, so the window redraws only then
        self.scan_count = 0
        self.nav_status = None    # the navigator's latest message (the reason if it failed)

    def p(self, name):
        return self.get_parameter(name).value

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.p('global_frame'), self.p('robot_frame'),
                                                Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    def on_plan(self, msg):
        self.plan = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def on_nav_status(self, msg):
        self.nav_status = msg.data
        if msg.data.startswith(('Replanning', 'No way around')):
            self.status = msg.data

    # ---- the lidar, turned into map coordinates --------------------------------
    def on_scan(self, msg):
        self.scan_count += 1
        if self.scan_count % 3:          # the lidar sends 10 scans a second: use 1 in 3
            return
        pose = self.robot_pose()
        if pose is None:
            return
        laser = (0.0, 0.0, 0.0)
        try:   # where the lidar sits on the robot (it is mounted turned round)
            t = self.tf_buffer.lookup_transform(self.p('robot_frame'), msg.header.frame_id, Time())
            laser = (t.transform.translation.x, t.transform.translation.y,
                     yaw_from_quaternion(t.transform.rotation))
        except TransformException:
            pass
        if self.layer is not None:
            self.layer.update(time.monotonic(), msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max, laser, pose)
        low, high = max(msg.range_min, 0.10), min(msg.range_max, DRAW_RANGE)
        # Only every 6th point is drawn: a few hundred canvas items are plenty
        self.scan_points = scan_points_in_map(msg.ranges, msg.angle_min, msg.angle_increment,
                                              low, high, laser, pose, stride=6)
        self.scan_version += 1

    def send_start(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p('global_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.initial_pose_pub.publish(msg)
        self.status = f'Start set to ({x:.2f}, {y:.2f}), facing {math.degrees(yaw):.0f} deg'

    # ---- the action: send goal -> accepted? -> feedback -> result --------
    def send_goal(self, x, y):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.status = 'Navigator not running (no /navigate_to_pose server)'
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.p('global_frame')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        self.status = f'Sending goal ({x:.2f}, {y:.2f}) ...'
        self.nav_status = None
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.status = 'Goal REJECTED (outside the map or inside a wall?)'
            return
        self.goal_handle = handle
        self.status = 'Goal ACCEPTED, driving ...'
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        est_time = fb.distance_remaining / 0.15
        replans = f', replans {fb.number_of_recoveries}' if fb.number_of_recoveries else ''
        self.status = (f'Driving... {fb.distance_remaining:.2f} m to go, '
                       f'est. {est_time:.0f} s remaining ({fb.navigation_time.sec} s elapsed)'
                       f'{replans}')

    def on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status = 'Goal COMPLETED: destination reached'
        elif status == GoalStatus.STATUS_CANCELED:
            self.status = 'Goal CANCELLED'
        else:
            reason = self.nav_status or 'see the navigator terminal for the reason'
            self.status = f'Goal FAILED. {reason}'
        self.goal_handle = None

    def cancel(self):
        self.cancel_all.call_async(CancelGoal.Request())
        self.cmd_pub.publish(Twist())      # and stop the wheels straight away
        self.status = 'Stop pressed'


class NavWindow:
    """The tkinter side: the map picture, the mouse and the buttons."""

    def __init__(self, ros):
        self.ros = ros
        self.grid = GridMap(ros.p('map_yaml'))
        self.grid.inflate(ros.p('inflation_radius'))
        # Live obstacles seen by the lidar, kept in memory (the map file is never changed)
        self.layer = LiveObstacleLayer(self.grid)
        ros.layer = self.layer
        self.scale = max(1, min(6, 700 // max(self.grid.width, self.grid.height)))
        self.start = None           # (x, y, yaw) chosen by the user
        self.goal = None            # (x, y)
        self.preview = []           # A* preview, list of (x, y)
        self.press = None           # where the mouse button went down
        self.drawn_version = -1     # scan_version of the lidar dots now on the canvas
        self.last_apply = 0.0       # when the live obstacles were last copied into the grid

        # --- Color Palette Constants (Ruby Garden) ---
        BG_CREAM = '#FFF9D6'       # Soft warm cream background
        TOOLBAR_PINK = '#FFDCDC'   # Soft pastel peach-pink header frame
        ROSE_ACCENT = '#D8A2A2'    # Muted dusty rose / terracotta
        SAGE_ACCENT = '#8EA66B'    # Fresh sage green
        TEXT_DARK = '#4A4A4A'      # Soft charcoal text for readability
        self.text_dark = TEXT_DARK

        self.root = tk.Tk()
        self.root.title('Raspbot Navigation Hub')
        self.root.configure(bg=BG_CREAM)

        # Top Control Toolbar with soft peach-pink background
        top = tk.Frame(self.root, bg=TOOLBAR_PINK, padx=14, pady=10)
        top.pack(fill='x')

        self.mode = tk.StringVar(value='goal')
        tk.Label(top, text='Click sets:', bg=TOOLBAR_PINK, fg=TEXT_DARK,
                 font=('Helvetica', 10, 'bold')).pack(side='left', padx=4)

        # Styled Radio Buttons matching the soft theme
        for text, val in [('Start', 'start'), ('Destination', 'goal')]:
            tk.Radiobutton(top, text=text, variable=self.mode, value=val,
                           bg=TOOLBAR_PINK, fg=TEXT_DARK, selectcolor=BG_CREAM,
                           activebackground=TOOLBAR_PINK, activeforeground=TEXT_DARK,
                           font=('Helvetica', 10)).pack(side='left', padx=4)

        # Helper to create sleek, clean rounded-style flat buttons
        def create_styled_button(parent, text, bg_color, command):
            return tk.Button(parent, text=text, bg=bg_color, fg='#ffffff',
                             relief='flat', activebackground=bg_color, activeforeground='#ffffff',
                             font=('Helvetica', 10, 'bold'), padx=14, pady=5, command=command)

        # Button Layout mapped cleanly across your custom palette
        create_styled_button(top, 'Go', SAGE_ACCENT, self.go).pack(side='left', padx=6)
        create_styled_button(top, 'Home', '#C98D8D', self.home).pack(side='left', padx=3)
        create_styled_button(top, 'Stop', ROSE_ACCENT, self.ros.cancel).pack(side='left', padx=6)
        create_styled_button(top, 'Clear', '#B5C49E', self.clear).pack(side='left', padx=3)

        # Map Canvas with crisp white frame and clean borderless finish
        w, h = self.grid.width * self.scale, self.grid.height * self.scale
        self.canvas = tk.Canvas(self.root, width=w, height=h, bg='#FFFFFF', highlightthickness=0)
        self.canvas.pack(padx=14, pady=14)

        self.image = self.make_image()
        self.canvas.create_image(0, 0, image=self.image, anchor='nw')
        self.canvas.bind('<ButtonPress-1>', self.on_press)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<Motion>', self.on_move)

        # Status and Info Readouts with refined spacing and typography
        self.status = tk.Label(self.root, anchor='w', bg=BG_CREAM, fg=TEXT_DARK,
                               font=('Helvetica', 11, 'bold'), padx=14)
        self.status.pack(fill='x', pady=(0, 2))

        self.info = tk.Label(self.root, anchor='w', bg=BG_CREAM, fg='#7A7A7A',
                             font=('Helvetica', 10), padx=14)
        self.info.pack(fill='x', pady=(0, 4))

        tk.Label(self.root, anchor='w', bg=BG_CREAM, fg='#8C8C8C', font=('Helvetica', 9), padx=14,
                 text='white = free  •  pink = inflation  •  black = wall  •  blue = robot/plan  '
                      '•  orange = preview  •  purple dots = lidar now  •  red = live obstacle'
                 ).pack(fill='x', pady=(0, 10))

        self.mouse = ''
        self.root.after(REFRESH_MS, self.refresh)

    # ---- map picture -------------------------------------------------------
    def make_image(self):
        """Draw the saved map once, cell by cell, into a tkinter picture."""
        rows = []
        for line in range(self.grid.height):
            row = self.grid.height - 1 - line            # top of the picture = top of map
            colours = []
            for col in range(self.grid.width):
                value = self.grid.cells[row][col]
                if value == OCCUPIED:
                    colour = COLOURS['wall']
                elif value != FREE:
                    colour = COLOURS['unknown']
                elif self.grid.blocked[row][col]:
                    colour = COLOURS['inflated']
                else:
                    colour = COLOURS['free']
                colours.append(colour)
            rows.append('{' + ' '.join(colours) + '}')
        image = tk.PhotoImage(width=self.grid.width, height=self.grid.height)
        image.put(' '.join(rows))
        return image.zoom(self.scale) if self.scale > 1 else image

    # ---- metres <-> screen pixels ---------------------------------------------
    def to_screen(self, x, y):
        col = (x - self.grid.origin_x) / self.grid.resolution
        row = (y - self.grid.origin_y) / self.grid.resolution
        return col * self.scale, (self.grid.height - row) * self.scale

    def to_world(self, px, py):
        x = self.grid.origin_x + px / self.scale * self.grid.resolution
        y = self.grid.origin_y + (self.grid.height - py / self.scale) * self.grid.resolution
        return x, y

    def robot_on_map(self, robot):
        """False when the robot's position is outside the free room: localisation is lost."""
        col, row = self.grid.world_to_cell(robot[0], robot[1])
        return (self.grid.inside(col, row)
                and self.grid.nearest_free(col, row, 10, live=False) is not None)

    # ---- mouse ---------------------------------------------------------------
    def on_move(self, event):
        x, y = self.to_world(event.x, event.y)
        self.mouse = f'mouse: x={x:.2f} m  y={y:.2f} m'

    def on_press(self, event):
        self.press = (event.x, event.y)

    def on_drag(self, event):
        if self.mode.get() == 'start' and self.press:
            self.canvas.delete('drag')
            self.canvas.create_line(*self.press, event.x, event.y, arrow='last', width=3,
                                    fill='green', tags='drag')

    def on_release(self, event):
        if self.press is None:
            return
        x, y = self.to_world(*self.press)
        dx, dy = event.x - self.press[0], self.press[1] - event.y   # screen y points down
        self.press = None
        self.canvas.delete('drag')
        if self.mode.get() == 'start':
            # Drag to show the heading. A plain click means "facing the mapping direction" (0 deg),
            # NOT the robot's current estimate: that estimate may be the very thing that is wrong.
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 5 else 0.0
            self.start = (x, y, yaw)
            self.ros.send_start(x, y, yaw)
        else:
            self.goal = (x, y)
            self.ros.status = f'Destination ({x:.2f}, {y:.2f}). Press Go to drive there.'
        self.update_preview()

    # ---- planning preview with the same A* as the navigator ---------------------
    def update_preview(self):
        self.preview = []
        robot = self.ros.robot_pose()
        begin = robot or self.start
        if begin is None or self.goal is None:
            return
        start = self.grid.nearest_free(*self.grid.world_to_cell(*begin[:2]), 6)
        goal = self.grid.nearest_free(*self.grid.world_to_cell(*self.goal), 4)
        if start is None or goal is None:
            self.ros.status = 'Start or destination is inside a wall'
            return
        cells, expanded = astar(self.grid, start, goal)
        if cells is None:
            self.ros.status = f'No path to the destination (A* expanded {expanded} cells)'
            return
        self.preview = [self.grid.cell_to_world(c, r) for c, r in simplify(self.grid, cells)]
        self.ros.status = (f'Preview: {path_length(self.grid, cells):.2f} m, A* expanded '
                           f'{expanded} cells. Press Go to drive there.')

    def home(self):
        """Send (0.0, 0.0) as the destination."""
        self.goal = (0.0, 0.0)
        self.ros.status = "Home destination (0.00, 0.00). Press Go to drive there."
        self.update_preview()

    def go(self):
        if self.goal is None:
            self.ros.status = 'Click a destination first'
            return
        self.ros.send_goal(*self.goal)

    def clear(self):
        self.goal, self.preview, self.ros.plan = None, [], []
        self.ros.status = 'Cleared'

    # ---- redraw 10 times a second -------------------------------------------
    def refresh(self):
        c = self.canvas
        c.delete('overlay')
        robot = self.ros.robot_pose()
        now = time.monotonic()
        if now - self.last_apply > 1.0:            # copy live obstacles into the grid (preview)
            self.last_apply = now
            self.layer.apply(now, robot[:2] if robot else None)
        if self.ros.scan_version != self.drawn_version:   # redraw the lidar only for a new scan
            self.drawn_version = self.ros.scan_version
            c.delete('live')
            self.draw_live(now)

        self.draw_path(self.preview, '#ff9900', dash=(4, 3))
        self.draw_path(self.ros.plan, '#1f5fd6')
        if self.start:
            self.draw_robot(self.start, '#2e9d2e')
        if self.goal:
            gx, gy = self.to_screen(*self.goal)
            c.create_line(gx - 7, gy - 7, gx + 7, gy + 7, fill='red', width=3, tags='overlay')
            c.create_line(gx - 7, gy + 7, gx + 7, gy - 7, fill='red', width=3, tags='overlay')
        lost = False
        if robot:
            self.draw_robot(robot, '#1f5fd6')
            where = f'robot: x={robot[0]:.2f} m  y={robot[1]:.2f} m  ' \
                    f'heading={math.degrees(robot[2]):.0f} deg'
            if not self.robot_on_map(robot):
                lost = True
                where += '   !! OUTSIDE THE MAP: localisation lost, set the Start again'
        else:
            where = 'robot: position unknown (no TF map -> base_link yet)'
        live = len(self.layer.obstacle_cells(now))
        self.status.config(text=self.ros.status)
        self.info.config(text=f'{where}     {self.mouse}     live obstacles: {live}',
                         fg='#c62828' if lost else '#7A7A7A')
        self.root.after(REFRESH_MS, self.refresh)

    def draw_live(self, now):
        """What the lidar sees right now: purple dots, and red squares for live obstacles."""
        c = self.canvas
        for x, y in self.ros.scan_points:
            sx, sy = self.to_screen(x, y)
            c.create_rectangle(sx - 1, sy - 1, sx + 1, sy + 1, fill=LIDAR_DOT, outline='',
                               tags='live')
        half = max(2, self.scale / 2)
        for col, row in self.layer.obstacle_cells(now):
            sx, sy = self.to_screen(*self.grid.cell_to_world(col, row))
            c.create_rectangle(sx - half, sy - half, sx + half, sy + half, fill=LIVE_OBSTACLE,
                               outline='', tags='live')

    def draw_path(self, points, colour, dash=None):
        if len(points) >= 2:
            xy = [v for p in points for v in self.to_screen(*p)]
            self.canvas.create_line(*xy, fill=colour, width=3, dash=dash, tags='overlay')

    def draw_robot(self, pose, colour):
        x, y = self.to_screen(pose[0], pose[1])
        r = max(5, 0.08 / self.grid.resolution * self.scale)
        self.canvas.create_oval(x - r, y - r, x + r, y + r, outline=colour, width=3, tags='overlay')
        hx, hy = x + 2 * r * math.cos(pose[2]), y - 2 * r * math.sin(pose[2])
        self.canvas.create_line(x, y, hx, hy, fill=colour, width=3, tags='overlay')


def main():
    rclpy.init()
    ros = NavClient()
    # ROS runs in a background thread, tkinter in the main thread
    spinner = threading.Thread(target=rclpy.spin, args=(ros,), daemon=True)
    spinner.start()
    window = NavWindow(ros)
    try:
        window.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
