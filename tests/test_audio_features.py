import math
import time
import unittest

import numpy as np

from audio.engine import AudioEngine, AudioSourceMode
from audio.features import FeatureAnalyzer
from audio.ring_buffer import AudioRingBuffer


def analyze_tone(frequency, amplitude=0.4, channels=1, seconds=0.45):
    rate, hop = 48000, 512
    analyzer = FeatureAnalyzer(rate, channels)
    output = None
    frame_count = int(seconds * rate / hop)
    for block in range(frame_count):
        t = (np.arange(hop) + block * hop) / rate
        signal = amplitude * np.sin(2.0 * math.pi * frequency * t)
        frames = signal[:, None]
        if channels == 2:
            frames = np.column_stack((signal, signal))
        output = analyzer.push(frames.astype(np.float32), block * hop / rate)
    return output


class RingBufferTests(unittest.TestCase):
    def test_overflow_discards_oldest_frames_without_changing_shape(self):
        ring = AudioRingBuffer(5, 2)
        ring.write(np.arange(8, dtype=np.float32).reshape(4, 2))
        ring.write(np.arange(8, 16, dtype=np.float32).reshape(4, 2))
        result = ring.read(10)
        self.assertEqual(result.shape, (5, 2))
        np.testing.assert_array_equal(result, np.arange(6, 16, dtype=np.float32).reshape(5, 2))
        self.assertEqual(ring.dropped_frames, 3)


class FeatureAnalyzerTests(unittest.TestCase):
    def test_generated_sines_select_the_documented_log_band(self):
        expected = {50: 0, 100: 0, 250: 2, 500: 3, 1000: 4, 2500: 5, 4000: 6, 10000: 7}
        for frequency, band in expected.items():
            with self.subTest(frequency=frequency):
                feature = analyze_tone(frequency)
                self.assertEqual(int(np.argmax(feature.bands)), band)
                self.assertGreater(feature.bands[band], 0.14)

    def test_stereo_balance_reports_left_center_and_right_only(self):
        rate, hop = 48000, 512
        for left_gain, right_gain, expected in ((1, 0, -1), (1, 1, 0), (0, 1, 1)):
            analyzer = FeatureAnalyzer(rate, 2)
            feature = None
            for block in range(12):
                t = (np.arange(hop) + block * hop) / rate
                tone = 0.4 * np.sin(2 * math.pi * 400 * t)
                feature = analyzer.push(np.column_stack((tone * left_gain, tone * right_gain)), block * hop / rate)
            self.assertAlmostEqual(feature.stereo_balance, expected, delta=0.08)

    def test_pitch_class_matches_across_octaves(self):
        a4 = analyze_tone(440.0)
        a5 = analyze_tone(880.0)
        self.assertEqual(int(np.argmax(a4.pitch_classes)), 9)
        self.assertEqual(int(np.argmax(a5.pitch_classes)), 9)
        self.assertGreater(a5.octave_position, a4.octave_position)

    def test_onset_refractory_prevents_duplicate_events(self):
        analyzer = FeatureAnalyzer(48000, 1)
        silence = np.zeros((1024, 1), dtype=np.float32)
        for index in range(5):
            analyzer.analyze(silence, index * 0.02)
        impulse = silence.copy()
        impulse[120:180, 0] = np.hanning(60).astype(np.float32)
        first = analyzer.analyze(impulse, 0.12)
        second = analyzer.analyze(impulse, 0.16)
        self.assertTrue(first.onset)
        self.assertFalse(second.onset)

    def test_steady_clicks_lock_tempo_within_four_beats(self):
        analyzer = FeatureAnalyzer(48000, 1)
        for onset_time in (0.0, 0.5, 1.0, 1.5):
            analyzer._onset_times.append(onset_time)
        tempo, confidence, phase = analyzer._tempo(1.5)
        self.assertAlmostEqual(tempo, 120.0, delta=0.5)
        self.assertGreater(confidence, 0.65)
        self.assertAlmostEqual(phase, 0.0, delta=0.01)
        _, stale_confidence, _ = analyzer._tempo(4.1)
        self.assertEqual(stale_confidence, 0.0)

    def test_white_noise_has_low_pitch_class_confidence(self):
        rng = np.random.default_rng(86)
        analyzer = FeatureAnalyzer(48000, 1)
        feature = None
        for block in range(20):
            noise = rng.normal(0.0, 0.2, 512).astype(np.float32)
            feature = analyzer.push(noise[:, None], block * 512 / 48000)
        self.assertLess(feature.pitch_confidence, 0.2)


class SimulatedEngineTests(unittest.TestCase):
    def test_demo_source_publishes_bounded_features_and_stops(self):
        engine = AudioEngine()
        try:
            engine.configure(AudioSourceMode.DEMO)
            deadline = time.monotonic() + 1.5
            frame = engine.snapshot()
            while frame.sequence == 0 and time.monotonic() < deadline:
                time.sleep(0.02)
                frame = engine.snapshot()
            self.assertGreater(frame.sequence, 0)
            self.assertTrue(all(0.0 <= value <= 1.0 for value in frame.system.bands))
            self.assertEqual(engine.status()["system"].state, "demo")
        finally:
            engine.shutdown()
        self.assertEqual(engine.mode, AudioSourceMode.OFF)


if __name__ == "__main__":
    unittest.main()
