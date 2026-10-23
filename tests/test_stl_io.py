import tempfile
import unittest
from pathlib import Path

import numpy as np

from lib.stl_io import (bbox, merge_binary_stls, read_binary_stl, write_box_faces,
                        write_box_stl, write_ground_bands, write_ground_plane)


class StlIoTests(unittest.TestCase):
    def test_box_roundtrip_and_bbox(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "box.stl"
            write_box_stl(path, [-1.0, 2.0, -3.0, 4.0, 0.0, 5.0])
            tri = read_binary_stl(path)
            self.assertEqual(tri.shape, (12, 3, 3))
            lo, hi = bbox([tri])
            np.testing.assert_allclose(lo, [-1.0, -3.0, 0.0])
            np.testing.assert_allclose(hi, [2.0, 4.0, 5.0])

    def test_merge(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = Path(tmp) / "a.stl"
            b = Path(tmp) / "b.stl"
            out = Path(tmp) / "out.stl"
            write_box_stl(a, [0, 1, 0, 1, 0, 1])
            write_box_stl(b, [2, 3, 2, 3, 2, 3])
            self.assertEqual(merge_binary_stls([a, b], out), 24)
            self.assertEqual(read_binary_stl(out).shape, (24, 3, 3))


    def test_single_ground_plane(self):
        with tempfile.TemporaryDirectory() as tmp:
            box = [-5.0, 5.0, -4.0, 4.0, 0.0, 3.0]
            ground = write_ground_plane(Path(tmp), box)
            tri = read_binary_stl(ground["path"])
            lo, hi = bbox([tri])
            np.testing.assert_allclose(lo, [-5.0, -4.0, 0.0])
            np.testing.assert_allclose(hi, [5.0, 4.0, 0.0])

    def test_domain_faces_and_ground_partition(self):
        with tempfile.TemporaryDirectory() as tmp:
            box = [-5.0, 5.0, -4.0, 4.0, 0.0, 3.0]
            walls = write_box_faces(Path(tmp) / "walls", box)
            self.assertEqual(len(walls), 5)
            bands = write_ground_bands(
                Path(tmp) / "ground", box,
                [-0.5, -0.3, 0.0], [1.0, 0.3, 0.01],
                [
                    {"size_m": 0.002, "outer_expand_from_plinth_m": 0.02},
                    {"size_m": 0.008, "outer_expand_from_plinth_m": 0.10},
                    {"size_m": 1.0, "outer_expand_from_plinth_m": None},
                ],
            )
            self.assertEqual(len(bands), 3)
            tri = np.concatenate([read_binary_stl(b["path"]) for b in bands], axis=0)
            lo, hi = bbox([tri])
            np.testing.assert_allclose(lo, [-5.0, -4.0, 0.0])
            np.testing.assert_allclose(hi, [5.0, 4.0, 0.0])
            area = 0.5 * np.linalg.norm(
                np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1
            ).sum()
            self.assertAlmostEqual(float(area), 80.0, places=6)


if __name__ == "__main__":
    unittest.main()
