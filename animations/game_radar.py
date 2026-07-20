"""Game Radar: honest stereo-biased transient localization."""

import math

import numpy as np

from .base import AudioAnimation, combine_features


class GameRadar(AudioAnimation):
    name = "RADAR"

    def __init__(self, topology):
        super().__init__(topology)
        self.event_age = 99.0
        self.seed = 0
        self.low_strength = 0.0
        self.high_strength = 0.0
        self.event_balance = 0.0

    def reset(self):
        super().reset()
        self.event_age = 99.0
        self.low_strength = self.high_strength = 0.0

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        del now
        feature = combine_features(frame)
        self.event_age += dt
        if self.is_new_frame(frame) and feature.onset:
            self.event_balance = feature.stereo_balance
            target = np.array([self.event_balance, 0.0, math.sqrt(max(0.0, 1.0 - self.event_balance ** 2))])
            if abs(self.event_balance) < 0.08:
                target = np.array([0.0, 0.0, 1.0])
            self.seed = int(np.argmax(self.topology.normals @ target)) if self.topology.count else 0
            self.low_strength = float(np.clip(np.mean(feature.bands[:3]) * feature.low_flux * gain * 2.2, 0.0, 1.0))
            self.high_strength = float(np.clip(np.mean(feature.bands[5:]) * feature.high_flux * gain * 2.4, 0.0, 1.0))
            self.event_age = 0.0
        distance = self.topology.distances_from(self.seed).astype(float) if self.topology.count else np.zeros(0)
        progress = self.event_age * 18.0 * reactivity
        front = np.exp(-0.5 * np.square((distance - progress) / (0.7 + feature.stereo_width)))
        decay = math.exp(-self.event_age / 0.72)
        broad = front * self.low_strength * decay
        needle = np.exp(-0.5 * np.square(distance / (0.8 + 2.2 * feature.stereo_width))) * self.high_strength * math.exp(-self.event_age / 0.18)
        tremor = feature.rms * 0.035 * (0.5 + 0.5 * np.sin(np.arange(self.topology.count) * 2.4 + frame.monotonic_time * 11.0))
        self.state.apex_scale[:] = 1.0 + 0.75 * needle + 0.22 * broad + tremor
        self.state.radial_offset[:] = self.topology.core_radius * (0.13 * broad + 0.025 * tremor)
        self.state.uniform_scale[:] = 1.0
        self.state.line_alpha[:] = 0.61 + 0.36 * np.maximum(broad, needle)
        self.state.line_width[:] = 1.35 + 2.1 * np.maximum(broad, needle)
        self.state.accent_mix[:] = np.clip(0.75 * front * decay, 0.0, 0.84)
        self.state.accent_hue[:] = np.mod((self.topology.azimuth + math.pi) / (2.0 * math.pi), 1.0)
        self.state.particle_rate[:] = needle * 9.0
        self.state.route = tuple(np.argsort(distance).tolist()) if len(distance) else ()
        self.state.route_energy = float(np.clip(max(self.low_strength, self.high_strength) * decay, 0.0, 1.0))
        return self.state
