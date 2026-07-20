import contextlib
import io
import tempfile
import unittest

import numpy as np

from animations.director import AnimationDirector, PROGRAM_NAMES
from animations.render import apply_render_state
from animations.signal_relay import SignalRelay
from animations.topology import FormationTopology
from audio.features import AudioFeatureFrame, SourceFeatures
from mastercontroller import MasterController


ACTIVE = SourceFeatures(
    active=True,
    rms=0.72,
    peak=0.91,
    bands=(0.9, 0.65, 0.35, 0.48, 0.55, 0.36, 0.82, 0.24),
    centroid=0.53,
    flux=0.86,
    onset=True,
    beat_phase=0.2,
    tempo_bpm=120.0,
    tempo_confidence=0.9,
    stereo_balance=0.6,
    stereo_width=0.35,
    pitch_classes=(0.0, 0.55, 0.0, 0.0, 0.4, 0.0, 0.82, 0.0, 0.0, 0.3, 0.0, 0.0),
    pitch_confidence=0.82,
    octave_position=0.64,
    spectral_flatness=0.12,
    low_flux=0.8,
    high_flux=0.75,
)


def active_frame(sequence=1):
    return AudioFeatureFrame(sequence, sequence / 60.0, ACTIVE, SourceFeatures.silence(), 21.0, 0)


class TopologyAndRenderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.controller = MasterController(self.directory.name)
        with contextlib.redirect_stdout(io.StringIO()):
            self.controller.init_star_formation(subdivisions=1, core_radius=3.0, spike_height=2.4)

    def tearDown(self):
        self.directory.cleanup()

    def test_star_topology_is_connected_with_three_shared_edge_neighbors(self):
        topology = FormationTopology(self.controller.pyramids)
        self.assertTrue(topology.shared_surface)
        self.assertEqual(topology.count, 80)
        self.assertEqual({len(neighbors) for neighbors in topology.adjacency}, {3})
        self.assertLess(int(topology.distances_from(0).max()), topology.count)

    def test_transient_tip_motion_preserves_all_star_base_points(self):
        director = AnimationDirector(self.controller.pyramids)
        director.select("CROWN")
        state = director.update(1.0, 1.0, active_frame())
        original_paths = [path.copy() for path in director.topology.home_paths]
        for index, original in enumerate(original_paths):
            transformed = apply_render_state(original, director.topology, index, state)
            np.testing.assert_allclose(transformed[:3], original[:3], atol=1e-9)
            np.testing.assert_array_equal(director.topology.home_paths[index], original)


class ProgramIntegrationTests(unittest.TestCase):
    def test_every_program_is_finite_and_bounded_across_all_formations(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = MasterController(directory)
            formations = (
                lambda: controller.init_edge_to_edge_pyramids(count=7),
                lambda: controller.init_spike_sphere_pyramids(count=24),
                lambda: controller.init_grid_pyramids(rows=4, cols=5),
                lambda: controller.init_star_formation(subdivisions=1),
                lambda: controller.init_globe_icosahedron(subdivisions=2),
            )
            with contextlib.redirect_stdout(io.StringIO()):
                for make_formation in formations:
                    make_formation()
                    director = AnimationDirector(controller.pyramids)
                    for program in PROGRAM_NAMES:
                        with self.subTest(count=len(controller.pyramids), program=program):
                            director.select(program)
                            state = director.update(0.3, 0.3, active_frame())
                            self.assertTrue(np.all(np.isfinite(state.apex_scale)))
                            self.assertTrue(np.all(np.isfinite(state.radial_offset)))
                            self.assertTrue(np.all((state.apex_scale >= 0.55) & (state.apex_scale <= 1.9)))
                            self.assertLessEqual(float(np.max(np.abs(state.radial_offset), initial=0.0)), 0.35 * director.topology.core_radius + 1e-9)

    def test_program_switch_crossfades_and_manual_none_returns_identity_immediately(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            controller = MasterController(directory)
            controller.init_star_formation(subdivisions=1)
            director = AnimationDirector(controller.pyramids)
            director.select("CROWN")
            crown = director.update(0.25, 0.25, active_frame()).copy()
            director.select("RADAR")
            transition = director.update(0.05, 0.30, active_frame(2))
            self.assertFalse(np.allclose(transition.apex_scale, 1.0))
            self.assertFalse(np.allclose(transition.apex_scale, crown.apex_scale))
            director.select(None)
            self.assertTrue(np.allclose(director.output.apex_scale, 1.0))
            self.assertTrue(np.allclose(director.output.radial_offset, 0.0))

    def test_every_program_returns_exactly_home_after_1_5_seconds_of_silence(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            controller = MasterController(directory)
            controller.init_star_formation(subdivisions=1)
        silence = SourceFeatures.silence()
        for program in PROGRAM_NAMES:
            with self.subTest(program=program):
                director = AnimationDirector(controller.pyramids)
                director.select(program)
                director.update(0.25, 0.0, active_frame())
                for tick in range(1, 91):
                    state = director.update(
                        1.0 / 60.0,
                        tick / 60.0,
                        AudioFeatureFrame(tick + 1, tick / 60.0, silence, silence),
                    )
                np.testing.assert_array_equal(state.apex_scale, np.ones(80))
                np.testing.assert_array_equal(state.uniform_scale, np.ones(80))
                np.testing.assert_array_equal(state.radial_offset, np.zeros(80))

    def test_signal_relay_fixed_step_is_render_rate_independent(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            controller = MasterController(directory)
            controller.init_star_formation(subdivisions=1)
            topology = FormationTopology(controller.pyramids)

        results = []
        for fps in (30, 60, 144):
            relay = SignalRelay(topology)
            frame = active_frame()
            for tick in range(fps):
                relay.update(1.0 / fps, tick / fps, frame)
            results.append(relay.displacement.copy())
        np.testing.assert_allclose(results[0], results[1], atol=1e-8)
        np.testing.assert_allclose(results[1], results[2], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
