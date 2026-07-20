import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mastercontroller import (
    MAX_GLOBE_SUBDIVISIONS,
    MasterController,
    generate_icosahedron_faces,
)
from pyramid import Pyramid


def transformed_apex_direction(pyramid):
    local_apex = np.array([0.0, pyramid.apex_height, 0.0], dtype=float)
    transformed = Pyramid.transform_path(
        [local_apex], pyramid.physics.position, pyramid.physics.rotation
    )[0]
    direction = transformed - pyramid.physics.position
    return direction / np.linalg.norm(direction)


class GeometryTests(unittest.TestCase):
    def test_globe_subdivision_is_bounded(self):
        maximum_faces = 20 * (4 ** MAX_GLOBE_SUBDIVISIONS)
        self.assertEqual(len(generate_icosahedron_faces(99)), maximum_faces)
        self.assertEqual(len(generate_icosahedron_faces(-4)), 20)

    def test_spike_sphere_apexes_point_outward_and_positions_stay_fixed(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = MasterController(export_directory=directory)
            with contextlib.redirect_stdout(io.StringIO()):
                controller.init_spike_sphere_pyramids(count=32, sphere_radius=4.0)

            original_positions = [p.physics.position.copy() for p in controller.pyramids]
            for pyramid in controller.pyramids:
                radial = pyramid.physics.position / np.linalg.norm(pyramid.physics.position)
                self.assertGreater(np.dot(transformed_apex_direction(pyramid), radial), 0.999)
                self.assertEqual(pyramid.physics.gravity, 0.0)

            controller.update(0.5, 0.5)
            for pyramid, original in zip(controller.pyramids, original_positions):
                np.testing.assert_allclose(pyramid.physics.position, original)

    def test_star_is_a_closed_edge_sharing_spike_sphere(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = MasterController(export_directory=directory)
            with contextlib.redirect_stdout(io.StringIO()):
                controller.init_star_formation(
                    subdivisions=1, core_radius=3.0, spike_height=2.4
                )

            self.assertEqual(len(controller.pyramids), 80)
            edge_counts = {}
            for pyramid in controller.pyramids:
                base = pyramid.local_path[:3]
                for first, second in ((base[0], base[1]), (base[1], base[2]), (base[2], base[0])):
                    endpoints = sorted(
                        (tuple(np.round(first, 8)), tuple(np.round(second, 8)))
                    )
                    edge = tuple(endpoints)
                    edge_counts[edge] = edge_counts.get(edge, 0) + 1

                face_center = sum(base) / 3.0
                face_normal = face_center / np.linalg.norm(face_center)
                apex = pyramid.local_path[4]
                apex_direction = apex / np.linalg.norm(apex)
                self.assertGreater(np.dot(apex_direction, face_normal), 0.999)
                self.assertGreater(np.linalg.norm(apex), 3.0)

            self.assertTrue(edge_counts)
            self.assertEqual(set(edge_counts.values()), {2})

    def test_wave_is_visual_offset_not_position_drift(self):
        pyramid = Pyramid()
        pyramid.physics.gravity = 0.0
        pyramid.physics.wave_axis_enable["y"] = True
        original = pyramid.physics.position.copy()

        for frame in range(600):
            pyramid.update(1.0 / 60.0, frame / 60.0)

        np.testing.assert_allclose(pyramid.physics.position, original)
        self.assertLessEqual(abs(pyramid.physics.wave_offset[1]), 0.5)


class ExportSynchronizationTests(unittest.TestCase):
    def managed_ids(self, directory):
        return {
            int(path.stem.removeprefix("pyramid_"))
            for path in Path(directory).glob("pyramid_*.txt")
            if path.stem.removeprefix("pyramid_").isdigit()
        }

    def test_exports_mirror_each_active_formation(self):
        with tempfile.TemporaryDirectory() as directory:
            sentinel = Path(directory, "notes.txt")
            sentinel.write_text("preserve me", encoding="utf-8")
            similarly_named = Path(directory, "pyramid_draft.txt")
            similarly_named.write_text("preserve me too", encoding="utf-8")

            controller = MasterController(export_directory=directory)
            with contextlib.redirect_stdout(io.StringIO()):
                controller.init_edge_to_edge_pyramids(count=7)
                self.assertEqual(self.managed_ids(directory), set(range(1, 8)))

                controller.init_spike_sphere_pyramids(count=24)
                self.assertEqual(self.managed_ids(directory), set(range(1, 25)))

                controller.init_grid_pyramids(rows=4, cols=5)
                self.assertEqual(self.managed_ids(directory), set(range(1, 21)))

                controller.init_globe_icosahedron(subdivisions=1)
                self.assertEqual(self.managed_ids(directory), set(range(1, 81)))

                controller.init_star_formation(subdivisions=1)
                self.assertEqual(self.managed_ids(directory), set(range(1, 81)))

                controller.init_grid_pyramids(rows=2, cols=3)
                self.assertEqual(self.managed_ids(directory), set(range(1, 7)))

                controller.clear_pyramids()
                self.assertEqual(self.managed_ids(directory), set())

            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve me")
            self.assertEqual(similarly_named.read_text(encoding="utf-8"), "preserve me too")


if __name__ == "__main__":
    unittest.main()
