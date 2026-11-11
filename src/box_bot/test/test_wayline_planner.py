"""Exercise the installed wayline plugin on a map with a blocked swath."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
import yaml


def test_wayline_rejoins_after_detour(tmp_path):
    domain = 175
    source = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((source/'config/nav2_minimal.yaml').read_text())
    costmap = config['global_costmap']['global_costmap']['ros__parameters']
    costmap.update(rolling_window=False, width=32, height=16, resolution=.25,
                   plugins=['static_layer', 'inflation_layer'])
    costmap['static_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer',
                                  map_subscribe_transient_local=True)
    params = tmp_path/'params.yaml'
    params.write_text(yaml.safe_dump(config))
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('wayline_test_map')
    publisher = node.create_publisher(OccupancyGrid, '/map', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = .25
    grid.info.width, grid.info.height = 128, 64
    grid.info.origin.orientation.w = 1.
    grid.info.origin.position.x, grid.info.origin.position.y = -2., -8.
    # Block x=8..10, y=-1..1 across the original wayline y=0.
    grid.data = [100 if 40 <= x < 48 and 28 <= y < 36 else 0
                 for y in range(grid.info.height) for x in range(grid.info.width)]
    publisher.publish(grid)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_footprint'
    transform.transform.rotation.w = 1.
    tf.sendTransform(transform)

    log_path = tmp_path/'planner.log'
    action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
    process = None

    def finish(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), log_path.read_text()
        return future.result()

    def pose(x, y):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.pose.position.x = float(x)
        result.pose.position.y = float(y)
        result.pose.orientation.w = 1.
        return result

    def plan(start, use_wayline, target=(25, 0), reference=(0, 0)):
        goal = ComputePathToPose.Goal(start=pose(*start), goal=pose(*target),
                                      use_start=True, planner_id='Wayline' if use_wayline else 'GridBased')
        if use_wayline:
            goal.viapoints = [pose(*reference)]
        handle = finish(action.send_goal_async(goal))
        assert handle.accepted, log_path.read_text()
        result = finish(handle.get_result_async()).result
        assert result.error_code == 0, result.error_msg
        return [p.pose.position for p in result.path.poses]

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                '/opt/ros/lyrical/lib/nav2_planner/planner_server', '--ros-args',
                '--params-file', str(params),
            ], env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=log, stderr=subprocess.STDOUT)
        assert lifecycle.wait_for_service(timeout_sec=10), log_path.read_text()
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert finish(lifecycle.call_async(request)).success, log_path.read_text()
        assert action.wait_for_server(timeout_sec=10)
        deadline = time.monotonic()+5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert publisher.get_subscription_count(), log_path.read_text()

        # A diagonal row in free space must stay exactly on the reference,
        # even though it cuts across the global grid at a shallow angle.
        diagonal = plan((0, 3), True, (25, 6), (0, 3))
        assert len(diagonal) > 100
        assert all(abs(p.y - (3 + .12 * p.x)) < 1e-9 for p in diagonal)
        assert (diagonal[-1].x, diagonal[-1].y) == (25., 6.)

        detour = plan((0, 0), True)
        assert any(8.25 < p.x < 9.75 and abs(p.y) >= 1.25 for p in detour)
        downstream = [(round(p.x, 2), round(p.y, 2)) for p in detour if 16 < p.x < 23]
        assert downstream and max(abs(y) for _, y in downstream) < 1e-9, downstream
        assert all(not (8 <= p.x < 10 and abs(p.y) < 1.25) for p in detour)

        # Start the next replan alongside the obstacle. The original reference
        # still brings the path back to the swath while a direct replan cuts across.
        offset = plan((10.5, 2.5), True)
        direct = plan((10.5, 2.5), False)
        assert math.isclose(offset[-1].x, 25., abs_tol=.1)
        assert any(14 < p.x < 16 and abs(p.y) < .4 for p in offset)
        assert all(abs(p.y) > .75 for p in direct if 14 < p.x < 16)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
