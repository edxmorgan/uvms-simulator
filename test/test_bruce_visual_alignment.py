"""Check Bruce's moving propellers against circular rings in its body mesh."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
BLUE = ROOT / "description/blue"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
XACRO = "{http://ros.org/wiki/xacro}"


def measured_mounts():
    description = ET.parse(BLUE / "xacro/bluerov2_heavy_reach/bruce_urdf.xacro")
    macro = description.getroot().find(f"{XACRO}macro")
    visual = macro.find("link/visual")
    origin = visual.find("origin")
    rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).as_matrix()
    offset = np.fromstring(origin.get("xyz"), sep=" ")
    scale = np.fromstring(visual.find("geometry/mesh").get("scale"), sep=" ")
    mesh = ET.parse(BLUE / "meshes/bluerov2_heavy_reach/BruceAssembly_colored.dae")
    array = mesh.find(".//c:source[@id='bruce_color_00_geometry_positions']/c:float_array", NS)
    vertices = np.fromstring(array.text, sep=" ").reshape(-1, 3)

    # Circular ring planes measured in the unscaled CAD coordinates. The windows
    # select one housing, excluding other black parts that share its material.
    c, s = np.cos(np.deg2rad(42)), np.sin(np.deg2rad(42))
    rings = [
        (1, [c, 0, s], 1.8363, [2.1, 3.46, .41]),
        (2, [c, 0, -s], 6.1812, [8.69, 3.46, .41]),
        (3, [c, 0, -s], 7.9007, [2.16, 3.46, -9.41]),
        (4, [c, 0, s], .1168, [8.63, 3.46, -9.41]),
        (5, [0, 1, 0], 5.7599, [-1.98, 5.76, -.17]),
        (6, [0, 1, 0], 5.7599, [12.77, 5.76, -.17]),
        (7, [0, 1, 0], 5.7599, [-1.98, 5.76, -8.29]),
        (8, [0, 1, 0], 5.7599, [12.77, 5.76, -8.29]),
    ]
    for number, axis, plane, window in rings:
        axis = np.asarray(axis, dtype=float)
        u = np.cross(axis, [0., 0., 1.])
        u /= np.linalg.norm(u)
        basis = np.stack([u, np.cross(axis, u), axis], axis=1)
        points = vertices[np.linalg.norm(vertices - window, axis=1) < 1.8]
        points = points[np.abs(points @ axis - plane) < .0001]
        if len(points) < 60:
            raise AssertionError(f"Thruster {number}: reference CAD ring not found")
        xy = (points @ basis)[:, :2]
        fit = np.linalg.lstsq(np.c_[2 * xy, np.ones(len(xy))], (xy ** 2).sum(1), rcond=None)[0]
        center = basis @ np.r_[fit[:2], plane]
        expected = rotation @ (scale * center) + offset
        mount = macro.find(f"{XACRO}t200_urdf[@id='{number}']/origin")
        actual = np.fromstring(mount.get("xyz"), sep=" ")
        actual_axis = Rotation.from_euler("xyz", np.fromstring(mount.get("rpy"), sep=" ")).as_matrix()[:, 2]
        yield number, expected, actual, rotation @ axis, actual_axis


class BruceVisualAlignmentTest(unittest.TestCase):
    def test_all_eight_propellers_are_centered(self):
        for number, expected, actual, _, _ in measured_mounts():
            with self.subTest(thruster=number):
                error = np.linalg.norm(expected - actual)
                self.assertLess(error, .0001, f"Blade origin is {error * 1000:.3f} mm from its CAD ring center")

    def test_propeller_axes_match_housings(self):
        for number, _, _, expected, actual in measured_mounts():
            with self.subTest(thruster=number):
                self.assertGreater(abs(np.dot(expected, actual)), .999999)


if __name__ == "__main__":
    unittest.main()
