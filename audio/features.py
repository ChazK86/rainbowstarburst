"""Realtime audio feature contracts and the NumPy analysis baseline."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math
import time

import numpy as np

from .normalization import ActivityGate, AdaptiveRange


BAND_EDGES = (
    (35.0, 70.0),
    (70.0, 140.0),
    (140.0, 300.0),
    (300.0, 700.0),
    (700.0, 1600.0),
    (1600.0, 3500.0),
    (3500.0, 7500.0),
    (7500.0, 16000.0),
)


@dataclass(frozen=True)
class SourceFeatures:
    active: bool = False
    rms: float = 0.0
    peak: float = 0.0
    bands: tuple[float, ...] = (0.0,) * 8
    centroid: float = 0.0
    flux: float = 0.0
    onset: bool = False
    beat_phase: float = 0.0
    tempo_bpm: float | None = None
    tempo_confidence: float = 0.0
    stereo_balance: float = 0.0
    stereo_width: float = 0.0
    pitch_classes: tuple[float, ...] = (0.0,) * 12
    pitch_confidence: float = 0.0
    octave_position: float = 0.5
    spectral_flatness: float = 1.0
    low_flux: float = 0.0
    high_flux: float = 0.0

    @classmethod
    def silence(cls) -> "SourceFeatures":
        return cls()

    def muted(self) -> "SourceFeatures":
        return replace(self, active=False, rms=0.0, peak=0.0, bands=(0.0,) * 8, onset=False)


@dataclass(frozen=True)
class AudioFeatureFrame:
    sequence: int
    monotonic_time: float
    system: SourceFeatures
    microphone: SourceFeatures
    capture_latency_ms: float = 0.0
    dropped_windows: int = 0

    @classmethod
    def silence(cls, sequence: int = 0) -> "AudioFeatureFrame":
        return cls(sequence, time.monotonic(), SourceFeatures.silence(), SourceFeatures.silence())


class FeatureAnalyzer:
    """Stateful FFT, onset, beat, stereo, and pitch-class analyzer."""

    def __init__(self, sample_rate=48000, channels=2, window_size=1024, hop_size=512):
        self.sample_rate = int(sample_rate)
        self.channels = max(1, int(channels))
        self.window_size = int(window_size)
        self.hop_size = int(hop_size)
        # Zero padding improves pitch-class bin placement without increasing
        # the capture window or its latency.
        self.fft_size = max(4096, self.window_size)
        self._window = np.hanning(self.window_size).astype(np.float32)
        self._history = np.zeros((self.window_size, self.channels), dtype=np.float32)
        self._received = 0
        self._previous_magnitude = np.zeros(self.fft_size // 2 + 1, dtype=float)
        self._last_time = None
        self._last_onset = -1e9
        self._onset_times = deque(maxlen=16)
        self._flux_history = deque(maxlen=160)
        self._rms_range = AdaptiveRange(minimum_ceiling=0.015)
        self._peak_range = AdaptiveRange(minimum_ceiling=0.025)
        self._band_ranges = [AdaptiveRange(minimum_ceiling=0.004) for _ in BAND_EDGES]
        self._gate = ActivityGate()
        self._frequency = np.fft.rfftfreq(self.fft_size, 1.0 / self.sample_rate)
        self._band_masks = [
            (self._frequency >= low) & (self._frequency < min(high, self.sample_rate / 2.0))
            for low, high in BAND_EDGES
        ]
        self._pitch_mask = (self._frequency >= 55.0) & (self._frequency <= min(5000.0, self.sample_rate / 2.0))

    def push(self, frames: np.ndarray, monotonic_time: float | None = None) -> SourceFeatures | None:
        values = np.asarray(frames, dtype=np.float32)
        if values.ndim == 1:
            values = values.reshape(-1, 1)
        if values.shape[1] != self.channels:
            if values.shape[1] == 1 and self.channels > 1:
                values = np.repeat(values, self.channels, axis=1)
            else:
                values = values[:, : self.channels]
        count = values.shape[0]
        if count >= self.window_size:
            self._history[:] = values[-self.window_size :]
        elif count:
            self._history[:-count] = self._history[count:]
            self._history[-count:] = values
        self._received += count
        if self._received < self.window_size:
            return None
        return self.analyze(self._history, monotonic_time=monotonic_time)

    def analyze(self, frames: np.ndarray, monotonic_time: float | None = None) -> SourceFeatures:
        now = time.monotonic() if monotonic_time is None else float(monotonic_time)
        dt = self.hop_size / float(self.sample_rate) if self._last_time is None else max(1e-4, now - self._last_time)
        self._last_time = now

        values = np.asarray(frames, dtype=np.float32)
        if values.ndim == 1:
            values = values[:, None]
        if values.shape[0] != self.window_size:
            fixed = np.zeros((self.window_size, values.shape[1]), dtype=np.float32)
            copy_count = min(self.window_size, values.shape[0])
            fixed[-copy_count:] = values[-copy_count:]
            values = fixed
        mono = values.mean(axis=1, dtype=np.float64)
        mono -= float(mono.mean())
        peak_raw = float(np.max(np.abs(values), initial=0.0))
        rms_raw = float(np.sqrt(np.mean(np.square(mono))))
        rms = self._rms_range.update(rms_raw, dt)
        peak = self._peak_range.update(peak_raw, dt)

        spectrum = np.fft.rfft(mono * self._window, n=self.fft_size)
        magnitude = np.abs(spectrum) * (2.0 / max(1.0, float(self._window.sum())))
        power = magnitude * magnitude
        raw_bands = []
        for mask in self._band_masks:
            raw_bands.append(float(np.sqrt(np.mean(power[mask]))) if np.any(mask) else 0.0)
        dominant_band_energy = max(max(raw_bands, default=0.0), 1e-12)
        band_values = tuple(
            float(np.clip(
                normalizer.update(value, dt)
                * math.sqrt(max(0.0, rms))
                * math.pow(value / dominant_band_energy, 0.65),
                0.0,
                1.0,
            ))
            for normalizer, value in zip(self._band_ranges, raw_bands)
        )

        useful = self._frequency <= min(16000.0, self.sample_rate / 2.0)
        mag_sum = float(magnitude[useful].sum())
        centroid_hz = (
            float(np.sum(self._frequency[useful] * magnitude[useful]) / mag_sum)
            if mag_sum > 1e-12 else 0.0
        )
        centroid = float(np.clip(centroid_hz / 16000.0, 0.0, 1.0))

        positive = np.maximum(magnitude - self._previous_magnitude, 0.0)
        previous_sum = float(self._previous_magnitude.sum())
        # A transition out of true silence still needs to be an onset.  Using a
        # small fraction of current energy as the denominator keeps that first
        # attack finite while retaining the usual previous-frame flux ratio.
        flux_denominator = max(1e-8, previous_sum, float(magnitude.sum()) * 0.05)
        flux_raw = float(positive.sum() / flux_denominator)
        low_mask = self._frequency < 300.0
        high_mask = self._frequency >= 1600.0
        low_flux_raw = float(positive[low_mask].sum() / max(
            1e-8,
            float(self._previous_magnitude[low_mask].sum()),
            float(magnitude[low_mask].sum()) * 0.05,
        ))
        high_flux_raw = float(positive[high_mask].sum() / max(
            1e-8,
            float(self._previous_magnitude[high_mask].sum()),
            float(magnitude[high_mask].sum()) * 0.05,
        ))
        self._previous_magnitude[:] = magnitude

        self._flux_history.append(flux_raw)
        flux_samples = np.fromiter(self._flux_history, dtype=float)
        median_flux = float(np.median(flux_samples)) if len(flux_samples) else 0.0
        mad = float(np.median(np.abs(flux_samples - median_flux))) if len(flux_samples) else 0.0
        threshold = max(0.08, median_flux + 3.0 * mad)
        onset = bool(
            len(flux_samples) >= 4
            and flux_raw > threshold
            and rms > 0.07
            and now - self._last_onset >= 0.08
        )
        if onset:
            self._last_onset = now
            self._onset_times.append(now)
        flux = float(np.clip(flux_raw / max(threshold, 1e-6), 0.0, 1.0))
        low_flux = float(np.clip(low_flux_raw / max(threshold, 1e-6), 0.0, 1.0))
        high_flux = float(np.clip(high_flux_raw / max(threshold, 1e-6), 0.0, 1.0))

        tempo_bpm, tempo_confidence, beat_phase = self._tempo(now)
        stereo_balance, stereo_width = self._stereo(values)
        flatness = self._flatness(magnitude[useful])
        pitch_classes, pitch_confidence, octave_position = self._pitch(power)
        voice_energy = sum(raw_bands[2:6])
        total_band_energy = sum(raw_bands) + 1e-12
        voice_ratio = float(np.clip(voice_energy / total_band_energy, 0.0, 1.0))
        active = self._gate.update(rms, voice_ratio, dt) and rms_raw > 2e-5

        return SourceFeatures(
            active=active,
            rms=rms,
            peak=peak,
            bands=band_values,
            centroid=centroid,
            flux=flux,
            onset=onset,
            beat_phase=beat_phase,
            tempo_bpm=tempo_bpm,
            tempo_confidence=tempo_confidence,
            stereo_balance=stereo_balance,
            stereo_width=stereo_width,
            pitch_classes=pitch_classes,
            pitch_confidence=pitch_confidence,
            octave_position=octave_position,
            spectral_flatness=flatness,
            low_flux=low_flux,
            high_flux=high_flux,
        )

    def _tempo(self, now: float) -> tuple[float | None, float, float]:
        if len(self._onset_times) < 3:
            return None, 0.0, 0.0
        intervals = np.diff(np.fromiter(self._onset_times, dtype=float))
        intervals = intervals[(intervals >= 0.25) & (intervals <= 1.5)]
        if len(intervals) < 2:
            return None, 0.0, 0.0
        period = float(np.median(intervals))
        consistency = 1.0 - float(np.clip(np.std(intervals) / max(period, 1e-6), 0.0, 1.0))
        age = now - self._onset_times[-1]
        freshness = float(np.clip(1.0 - max(0.0, age - period) / 2.0, 0.0, 1.0))
        # Four accepted onsets yield three intervals, enough to establish a
        # stable pulse while still rejecting inconsistent speech timing.
        confidence = consistency * min(1.0, len(intervals) / 3.0) * freshness
        return 60.0 / period, confidence, float((age / period) % 1.0)

    @staticmethod
    def _stereo(values: np.ndarray) -> tuple[float, float]:
        if values.shape[1] < 2:
            return 0.0, 0.0
        left = values[:, 0].astype(float)
        right = values[:, 1].astype(float)
        left_rms = float(np.sqrt(np.mean(left * left)))
        right_rms = float(np.sqrt(np.mean(right * right)))
        total = left_rms + right_rms
        if total <= 1e-9:
            return 0.0, 0.0
        balance = (right_rms - left_rms) / total
        difference = float(np.sqrt(np.mean(np.square(left - right))))
        width = float(np.clip(difference / max(total, 1e-9), 0.0, 1.0))
        return float(np.clip(balance, -1.0, 1.0)), width

    @staticmethod
    def _flatness(magnitude: np.ndarray) -> float:
        values = np.maximum(np.asarray(magnitude, dtype=float), 1e-12)
        if not np.any(values > 1e-10):
            return 1.0
        return float(np.clip(np.exp(np.mean(np.log(values))) / np.mean(values), 0.0, 1.0))

    def _pitch(self, power: np.ndarray) -> tuple[tuple[float, ...], float, float]:
        frequencies = self._frequency[self._pitch_mask]
        energies = power[self._pitch_mask]
        total = float(energies.sum())
        if total <= 1e-12:
            return (0.0,) * 12, 0.0, 0.5
        midi = 69.0 + 12.0 * np.log2(frequencies / 440.0)
        nearest = np.rint(midi)
        tuning_weight = np.exp(-0.5 * np.square((midi - nearest) / 0.24))
        weighted = energies * tuning_weight
        classes = np.zeros(12, dtype=float)
        np.add.at(classes, np.mod(nearest.astype(int), 12), weighted)
        class_total = float(classes.sum())
        if class_total > 1e-12:
            classes /= class_total
        uniform = 1.0 / 12.0
        confidence = float(np.clip((float(classes.max()) - uniform) / (1.0 - uniform), 0.0, 1.0))
        octaves = np.clip((midi - 36.0) / 60.0, 0.0, 1.0)
        octave_position = float(np.sum(octaves * energies) / total)
        return tuple(float(x) for x in classes), confidence, octave_position
