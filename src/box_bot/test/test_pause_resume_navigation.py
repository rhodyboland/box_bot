"""Exercise the actual GPS BT's pause/resume with fake planning/control actions."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose, Wait
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('cancel_delay', [0.0, .15])
def test_gps_bt_pause_cancels_control_and_resume_keeps_mission(tmp_path, cancel_delay):
    rclpy.init(domain_id=177)
    node = rclpy.create_node('fake_navigation_services')
    group = ReentrantCallbackGroup()
    stopped = threading.Event()
    following, canceled, references = [], [], []

    def compute(handle):
        references.append((handle.request.planner_id, handle.request.viapoints))
        result = ComputePathToPose.Result()
        result.path.header.frame_id = 'map'
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.orientation.w = 1.
        result.path.poses = [start, handle.request.goal]
        handle.succeed()
        return result

    def follow(handle):
        following.append(handle)
        while not stopped.wait(.02):
            if handle.is_cancel_requested:
                time.sleep(cancel_delay)
                canceled.append(handle)
                handle.canceled()
                return FollowPath.Result()
        handle.abort()
        return FollowPath.Result()

    def wait(handle):
        handle.succeed()
        return Wait.Result()

    servers = [ActionServer(node, ComputePathToPose, '/compute_path_to_pose', compute, callback_group=group),
               ActionServer(node, FollowPath, '/follow_path', follow, callback_group=group,
                            cancel_callback=lambda request: CancelResponse.ACCEPT),
               ActionServer(node, Wait, '/wait', wait, callback_group=group)]
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.transform.rotation.w = 1.
    broadcaster.sendTransform(transform)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    source = Path(__file__).resolve().parents[1]
    log_path = tmp_path / 'navigator.log'
    navigation = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    process = None

    def until(predicate, seconds=10):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():
                return
            time.sleep(.02)
        raise AssertionError(log_path.read_text())

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(request)
        until(future.done)
        assert future.result().success, log_path.read_text()

    with log_path.open('w') as log:
        try:
            process = subprocess.Popen([
                '/opt/ros/lyrical/lib/nav2_bt_navigator/bt_navigator', '--ros-args',
                '--params-file', str(source/'config/nav2_minimal.yaml'),
                '-p', 'navigators:=[navigate_to_pose]',
                '-p', 'default_nav_to_pose_bt_xml:='+str(source/'behavior_trees/gps_nav_to_pose.xml'),
            ], env=dict(os.environ, ROS_DOMAIN_ID='177'), stdout=log, stderr=subprocess.STDOUT)
            lifecycle = node.create_client(ChangeState, '/bt_navigator/change_state')
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                call(lifecycle, request)
            assert navigation.wait_for_server(timeout_sec=10)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x = 2.
            goal.pose.pose.orientation.w = 1.
            future = navigation.send_goal_async(goal)
            until(future.done)
            handle = future.result()
            assert handle.accepted
            result = handle.get_result_async()
            until(lambda: len(following) == 1)
            pause = node.create_client(Trigger, '/traction_recovery/pause_navigation')
            resume = node.create_client(Trigger, '/traction_recovery/resume_navigation')
            call(pause, Trigger.Request())
            until(lambda: len(canceled) == 1)
            time.sleep(.2)
            assert not result.done()
            transform.transform.translation.y = 1.
            broadcaster.sendTransform(transform)
            call(resume, Trigger.Request())
            until(lambda: len(following) == 2)
            assert len(references) >= 2
            assert all(planner == 'Wayline' and len(points) == 1 and points[0].pose.position.y == 0.
                       for planner, points in references)
            assert not result.done()
            handle.cancel_goal_async()
            until(result.done)
            assert result.result().status == 5
            assert 'Failed to get result for follow_path in node halt!' not in log_path.read_text()
        finally:
            stopped.set()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            navigation.destroy()
            for server in servers:
                server.destroy()
            node.destroy_node()
            rclpy.shutdown()
