"""ROS 2 node: tell Cartographer where the robot starts on the saved map.

Cartographer (localisation mode) has no "2D Pose Estimate" of its own. This
node listens to /initialpose (from the GUI or from RViz) and:
  1. asks Cartographer which trajectories exist (/get_trajectory_states)
  2. finishes the current, active one       (/finish_trajectory)
  3. starts a new one at the given pose     (/start_trajectory)
"""

import math
import os
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class SetStart(Node):

    def __init__(self):
        super().__init__('set_start')
        default_dir = os.path.join(get_package_share_directory('raspbot_mapping'), 'config')
        self.declare_parameter('configuration_directory', default_dir)
        self.declare_parameter('configuration_basename', 'cartographer_localization.lua')

        group = ReentrantCallbackGroup()
        self.get_states = self.create_client(GetTrajectoryStates, 'get_trajectory_states',
                                             callback_group=group)
        self.finish = self.create_client(FinishTrajectory, 'finish_trajectory',
                                         callback_group=group)
        self.start = self.create_client(StartTrajectory, 'start_trajectory',
                                        callback_group=group)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose,
                                 10, callback_group=group)
        self.get_logger().info('set_start ready: waiting for /initialpose')

    def call(self, client, request):
        """Call a service and wait for the answer (at most 5 s)."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service {client.srv_name} not available. '
                                    'Is Cartographer running?')
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0):
            self.get_logger().error(f'No answer from {client.srv_name}')
            return None
        return future.result()

    def on_initial_pose(self, msg):
        p = msg.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.get_logger().info(f'New start: x={p.position.x:.2f} y={p.position.y:.2f} '
                               f'yaw={math.degrees(yaw):.0f} deg')

        states = self.call(self.get_states, GetTrajectoryStates.Request())
        if states is None:
            return
        ids = states.trajectory_states.trajectory_id
        codes = states.trajectory_states.trajectory_state
        for trajectory_id, state in zip(ids, codes):
            if state == TrajectoryStates.ACTIVE:
                self.call(self.finish, FinishTrajectory.Request(trajectory_id=trajectory_id))

        request = StartTrajectory.Request()
        request.configuration_directory = self.get_parameter('configuration_directory').value
        request.configuration_basename = self.get_parameter('configuration_basename').value
        request.use_initial_pose = True
        request.initial_pose = p
        request.relative_to_trajectory_id = 0      # the saved map's trajectory
        answer = self.call(self.start, request)
        if answer is not None:
            self.get_logger().info(f'Cartographer: {answer.status.message} '
                                   f'(trajectory {answer.trajectory_id})')


def main():
    rclpy.init()
    node = SetStart()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
