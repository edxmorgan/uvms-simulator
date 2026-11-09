"""Exercise the onboard camera projection without starting ROS or a GPU renderer."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time

from bringup.sim_camera_renderer import SimCameraRendererNode


class CameraClippingTests(unittest.TestCase):
    def test_close_arm_geometry_is_not_clipped_at_ten_centimeters(self):
        renderer = Mock()
        renderer.render_to_image.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        node = SimpleNamespace(
            _lookup_matrix=lambda frame: np.eye(4),
            renderer_backend="open3d",
            open3d_renderer=renderer,
            horizontal_fov_deg=75.0,
            _apply_underwater_effect=lambda image, depth: image,
            width=3,
            height=2,
        )
        stamp = Time(sec=1)
        message = SimCameraRendererNode.render_one(node, "robot_1_", stamp)
        args, kwargs = renderer.setup_camera.call_args
        self.assertEqual(kwargs, {"near_clip": 0.01})
        self.assertEqual(args[0], 75.0)
        np.testing.assert_array_equal(args[1], [0, 0, 1])
        np.testing.assert_array_equal(args[2], [0, 0, 0])
        np.testing.assert_array_equal(args[3], [0, -1, 0])
        self.assertEqual(message.header.frame_id, "robot_1_camera_link")
        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual((message.width, message.height, len(message.data)), (3, 2, 18))

    def test_physical_camera_buffer_uses_plant_pose_and_fixed_geometry(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer
        physical = SimpleNamespace(tf_buffer=Buffer(), world_frame='world')
        body = TransformStamped()
        body.header.frame_id, body.child_frame_id = 'world', 'robot_1_base_link'
        body.header.stamp = Time(sec=1)
        body.transform.translation.x = 2.
        body.transform.rotation.w = 1.
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'robot_1_base_link', 'robot_1_camera_link'
        mount.transform.translation.x = .21
        mount.transform.rotation.w = 1.
        SimCameraRendererNode._static_tf_callback(physical, TFMessage(transforms=[mount]))
        # Fixed geometry alone must not fabricate a camera pose.
        self.assertIsNone(SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link'))
        SimCameraRendererNode._physical_tf_callback(physical, TFMessage(transforms=[body]))
        pose = SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link')
        np.testing.assert_allclose(pose[:3, 3], [2.21, 0., 0.])
        # An unrelated operational estimate lives in a different TF buffer.
        operational = Buffer()
        body.transform.translation.x = 20.
        operational.set_transform(body, 'ekf')
        np.testing.assert_allclose(
            SimCameraRendererNode._lookup_matrix(physical, 'robot_1_camera_link')[:3, 3],
            [2.21, 0., 0.])


if __name__ == "__main__":
    unittest.main()
