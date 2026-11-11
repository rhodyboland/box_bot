"""A late old result must not block the next waypoint acknowledgement or cancel."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowWaypoints, NavigateToPose


def test_preempted_result_before_next_goal_ack_then_cancel(tmp_path):
    rclpy.init(domain_id=192)
    node = rclpy.create_node('fake_waypoint_navigation')
    group = ReentrantCallbackGroup()
    old_result = threading.Event()
    stop = threading.Event()
    nav_count = [0]
    executing = []

    def accept(request):
        nav_count[0] += 1
        if nav_count[0] == 2:
            old_result.set()
            time.sleep(.5)  # Old result is delivered before this acknowledgement.
        return GoalResponse.ACCEPT

    def execute(handle):
        index = len(executing)
        executing.append(handle)
        if index == 1:
            handle.succeed()
            return NavigateToPose.Result()
        while not stop.wait(.02):
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            if index == 0 and old_result.is_set():
                handle.abort()
                return NavigateToPose.Result()
        handle.abort()
        return NavigateToPose.Result()

    server = ActionServer(node, NavigateToPose, '/navigate_to_pose', execute,
                          goal_callback=accept, cancel_callback=lambda request: CancelResponse.ACCEPT,
                          callback_group=group)
    client = ActionClient(node, FollowWaypoints, '/follow_waypoints')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log_path = tmp_path/'waypoint.log'
    process = None

    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():
                return
            time.sleep(.02)
        raise AssertionError(log_path.read_text())

    def goal(x):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        request = FollowWaypoints.Goal()
        request.poses = [pose]
        future = client.send_goal_async(request)
        until(future.done)
        assert future.result().accepted
        return future.result()

    with log_path.open('w') as log:
        try:
            prefix = Path(get_package_prefix('nav2_launcher'))
            params = Path(__file__).resolve().parents[2]/'box_bot/config/nav2_minimal.yaml'
            process = subprocess.Popen([
                str(prefix/'lib/nav2_launcher/boxbot_waypoint_follower'), '--ros-args',
                '--params-file', str(params)], env=dict(os.environ, ROS_DOMAIN_ID='192'),
                stdout=log, stderr=subprocess.STDOUT)
            lifecycle = node.create_client(ChangeState, '/waypoint_follower/change_state')
            assert lifecycle.wait_for_service(timeout_sec=10)
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = lifecycle.call_async(request)
                until(future.done)
                assert future.result().success, log_path.read_text()
            assert client.wait_for_server(timeout_sec=5)
            first = goal(1.)
            until(lambda: len(executing) == 1)
            second = goal(2.)
            second_result = second.get_result_async()
            until(second_result.done)
            assert second_result.result().status == 4
            third = goal(3.)
            until(lambda: len(executing) == 3)
            third_result = third.get_result_async()
            third.cancel_goal_async()
            until(third_result.done)
            assert third_result.result().status == 5
        finally:
            stop.set()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            client.destroy()
            server.destroy()
            node.destroy_node()
            rclpy.shutdown()
