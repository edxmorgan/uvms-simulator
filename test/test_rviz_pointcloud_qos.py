from pathlib import Path

import yaml

from bringup.utils.rviz_utils import rviz_file_configure


def test_generated_pointcloud_displays_match_publishers(tmp_path):
    template = Path(__file__).resolve().parents[1] / "description/rviz/rviz.rviz"
    output = tmp_path / "generated.rviz"
    rviz_file_configure(
        False, False, ["robot_1_"], ["robot_1_base_link"], [1],
        str(template), str(output), "interactive", world_frame="base_link",
    )
    config = yaml.safe_load(output.read_text())
    planned = next(d for d in config['Visualization Manager']['Displays']
                   if d.get('Name') == 'planned waypoints')
    assert planned['Topic']['Depth'] == 100
    assert planned['Enabled']
    clouds = {
        display["Topic"]["Value"]: display
        for display in config["Visualization Manager"]["Displays"]
        if display.get("Class") == "rviz_default_plugins/PointCloud2"
    }
    for topic in ("/env_voxels_cloud", "/dynamic_voxels_cloud", "/base_pointcloud", "/workspace_pointcloud"):
        assert clouds[topic]["Enabled"] is True
        is_voxel = topic in ("/env_voxels_cloud", "/dynamic_voxels_cloud")
        reliability = "Reliable" if is_voxel else "Best Effort"
        assert clouds[topic]["Topic"]["Reliability Policy"] == reliability
        assert clouds[topic]["Topic"]["Durability Policy"] == ("Transient Local" if is_voxel else "Volatile")
        if is_voxel:
            assert clouds[topic]['Style'] == 'Flat Squares'
            assert clouds[topic]['Size (m)'] == 0.01


def test_planned_marker_burst_survives_rviz_subscription_queue():
    import time
    import uuid
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from visualization_msgs.msg import Marker
    from simlab.planner_markers import PathPlanner
    from simlab.vehicle_waypoint_mission import VehicleWaypointViz
    from bringup.utils.rviz_utils import rviz_contact_markers_display

    config = {'Visualization Manager': {'Displays': []}}
    rviz_contact_markers_display(config, '/planned_waypoints_marker', 'planned waypoints', depth=100)
    depth = config['Visualization Manager']['Displays'][0]['Topic']['Depth']
    context = Context()
    rclpy.init(context=context, domain_id=167)
    node = rclpy.create_node('marker_burst_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        received = []
        topic = '/marker_burst_' + uuid.uuid4().hex
        pub = node.create_publisher(Marker, topic, QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub = node.create_subscription(Marker, topic, received.append, QoSProfile(depth=depth))
        deadline = time.monotonic() + 3.
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        assert pub.get_subscription_count() == 1
        planner = PathPlanner(pub, ns='planner/robot_1_', base_id=0)
        mission = VehicleWaypointViz(pub, ns='vehicle_waypoints/robot_1_', base_id=2000)
        stamp = node.get_clock().now().to_msg()
        planner.update_path_viz(stamp, 'world', np.array([[0., 0., 0.], [8., 0., 0.]]))
        mission.clear(stamp, 'world')
        planner.update_target_viz(stamp, 'world', [1., 0., 0.], quat_wxyz=[1., 0., 0., 0.])
        # Reproduce a busy display: all six messages arrive before consumption.
        time.sleep(.15)
        deadline = time.monotonic() + 2.
        while len(received) < 6 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert len(received) == 6
        assert any(m.ns == planner.ns and m.id == 0 and m.action == Marker.ADD for m in received)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
