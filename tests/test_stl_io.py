import tempfile
import unittest
from pathlib import Path

import numpy as np

from lib.stl_io import bbox, merge_binary_stls, read_binary_stl, write_box_stl


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


if __name__ == "__main__":
    unittest.main()
