"""Shared animation contracts and bounded vector state."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from audio.features import AudioFeatureFrame


@dataclass
class RenderStateBatch:
    radial_offset: np.ndarray
    apex_scale: np.ndarray
    uniform_scale: np.ndarray
    rotation_offset: np.ndarray
    line_alpha: np.ndarray
    line_width: np.ndarray
    accent_mix: np.ndarray
    accent_hue: np.ndarray
    particle_rate: np.ndarray
    route: tuple[int, ...] = field(default_factory=tuple)
    route_energy: float = 0.0

    @classmethod
    def identity(cls, count: int) -> "RenderStateBatch":
        n = max(0, int(count))
        return cls(
            radial_offset=np.zeros(n, dtype=float),
            apex_scale=np.ones(n, dtype=float),
            uniform_scale=np.ones(n, dtype=float),
            rotation_offset=np.zeros((n, 3), dtype=float),
            line_alpha=np.full(n, 0.68, dtype=float),
            line_width=np.full(n, 1.5, dtype=float),
            accent_mix=np.zeros(n, dtype=float),
            accent_hue=np.zeros(n, dtype=float),
            particle_rate=np.zeros(n, dtype=float),
        )

    def copy(self) -> "RenderStateBatch":
        return RenderStateBatch(
            self.radial_offset.copy(),
            self.apex_scale.copy(),
            self.uniform_scale.copy(),
            self.rotation_offset.copy(),
            self.line_alpha.copy(),
            self.line_width.copy(),
            self.accent_mix.copy(),
            self.accent_hue.copy(),
            self.particle_rate.copy(),
            tuple(self.route),
            float(self.route_energy),
        )

    def clamp(self, core_radius: float, reduced_motion=False) -> "RenderStateBatch":
        motion = 0.32 if reduced_motion else 1.0
        self.radial_offset[:] = np.clip(self.radial_offset * motion, -0.35 * core_radius, 0.35 * core_radius)
        self.apex_scale[:] = 1.0 + np.clip((self.apex_scale - 1.0) * motion, -0.45, 0.9)
        self.uniform_scale[:] = 1.0 + np.clip((self.uniform_scale - 1.0) * motion, -0.18, 0.22)
        self.rotation_offset[:] = np.clip(self.rotation_offset * motion, -18.0, 18.0)
        self.line_alpha[:] = np.clip(self.line_alpha, 0.2, 1.0)
        self.line_width[:] = np.clip(self.line_width, 1.0, 4.5)
        self.accent_mix[:] = np.clip(self.accent_mix, 0.0, 0.88)
        self.accent_hue[:] = np.mod(self.accent_hue, 1.0)
        self.particle_rate[:] = np.clip(self.particle_rate * motion, 0.0, 30.0)
        self.route_energy = float(np.clip(self.route_energy, 0.0, 1.0))
        return self

    @classmethod
    def blend(cls, first: "RenderStateBatch", second: "RenderStateBatch", amount: float) -> "RenderStateBatch":
        t = float(np.clip(amount, 0.0, 1.0))

        def mix(a, b):
            return a + (b - a) * t

        return cls(
            mix(first.radial_offset, second.radial_offset),
            mix(first.apex_scale, second.apex_scale),
            mix(first.uniform_scale, second.uniform_scale),
            mix(first.rotation_offset, second.rotation_offset),
            mix(first.line_alpha, second.line_alpha),
            mix(first.line_width, second.line_width),
            mix(first.accent_mix, second.accent_mix),
            mix(first.accent_hue, second.accent_hue),
            mix(first.particle_rate, second.particle_rate),
            second.route if t >= 0.5 else first.route,
            float(first.route_energy + (second.route_energy - first.route_energy) * t),
        )


@dataclass(frozen=True)
class CombinedFeatures:
    active: bool
    rms: float
    peak: float
    bands: np.ndarray
    centroid: float
    flux: float
    onset: bool
    beat_phase: float
    tempo_confidence: float
    stereo_balance: float
    stereo_width: float
    pitch_classes: np.ndarray
    pitch_confidence: float
    octave_position: float
    flatness: float
    low_flux: float
    high_flux: float


def combine_features(frame: AudioFeatureFrame) -> CombinedFeatures:
    system, microphone = frame.system, frame.microphone
    weights = np.array([system.rms, microphone.rms], dtype=float)
    total = float(weights.sum())
    if total <= 1e-9:
        weights[:] = 0.5
    else:
        weights /= total
    bands = np.maximum(np.asarray(system.bands), np.asarray(microphone.bands))
    pitches = weights[0] * np.asarray(system.pitch_classes) + weights[1] * np.asarray(microphone.pitch_classes)
    dominant = system if system.rms >= microphone.rms else microphone
    return CombinedFeatures(
        active=system.active or microphone.active,
        rms=max(system.rms, microphone.rms),
        peak=max(system.peak, microphone.peak),
        bands=bands,
        centroid=float(weights[0] * system.centroid + weights[1] * microphone.centroid),
        flux=max(system.flux, microphone.flux),
        onset=system.onset or microphone.onset,
        beat_phase=dominant.beat_phase,
        tempo_confidence=max(system.tempo_confidence, microphone.tempo_confidence),
        stereo_balance=dominant.stereo_balance,
        stereo_width=dominant.stereo_width,
        pitch_classes=pitches,
        pitch_confidence=max(system.pitch_confidence, microphone.pitch_confidence),
        octave_position=float(weights[0] * system.octave_position + weights[1] * microphone.octave_position),
        flatness=float(weights[0] * system.spectral_flatness + weights[1] * microphone.spectral_flatness),
        low_flux=max(system.low_flux, microphone.low_flux),
        high_flux=max(system.high_flux, microphone.high_flux),
    )


def smooth(current: np.ndarray, target: np.ndarray, dt: float, attack=0.055, release=0.32) -> None:
    rising = target > current
    attack_alpha = 1.0 - math.exp(-max(0.0, dt) / max(1e-4, attack))
    release_alpha = 1.0 - math.exp(-max(0.0, dt) / max(1e-4, release))
    alpha = np.where(rising, attack_alpha, release_alpha)
    current += (target - current) * alpha


class AudioAnimation:
    name = "BASE"

    def __init__(self, topology):
        self.topology = topology
        self.state = RenderStateBatch.identity(topology.count)
        self.last_sequence = -1

    def reset(self) -> None:
        self.state = RenderStateBatch.identity(self.topology.count)
        self.last_sequence = -1

    def is_new_frame(self, frame: AudioFeatureFrame) -> bool:
        if frame.sequence == self.last_sequence:
            return False
        self.last_sequence = frame.sequence
        return True

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0) -> RenderStateBatch:
        raise NotImplementedError
