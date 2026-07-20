"""Harmonic Aurora: pitch-class constellations with noise-aware coherence."""

import math

import numpy as np

from .base import AudioAnimation, combine_features, smooth


class HarmonicAurora(AudioAnimation):
    name = "AURORA"

    def __init__(self, topology):
        super().__init__(topology)
        self.energy = np.zeros(topology.count, dtype=float)
        self.previous_class = 0
        self.route = ()

    def reset(self):
        super().reset()
        self.energy[:] = 0.0
        self.previous_class = 0
        self.route = ()

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        del now
        feature = combine_features(frame)
        coherence = np.clip(feature.pitch_confidence * (1.0 - feature.flatness), 0.0, 1.0)
        localized = self.topology.pitch_influence @ feature.pitch_classes
        diffuse = np.full(self.topology.count, feature.rms * 0.22)
        vertical = np.exp(-0.5 * np.square((self.topology.latitude - (feature.octave_position * 2.0 - 1.0)) / 0.5)) if self.topology.count else np.zeros(0)
        target = np.clip((coherence * localized * (0.45 + 0.85 * vertical) + (1.0 - coherence) * diffuse) * gain * 2.8, 0.0, 1.0)
        smooth(self.energy, target, dt, attack=0.06 / reactivity, release=0.42 / reactivity)
        dominant_class = int(np.argmax(feature.pitch_classes)) if len(feature.pitch_classes) else 0
        if self.is_new_frame(frame) and dominant_class != self.previous_class and coherence > 0.12 and self.topology.count:
            old_node = int(np.argmax(self.topology.pitch_influence[:, self.previous_class]))
            new_node = int(np.argmax(self.topology.pitch_influence[:, dominant_class]))
            self.route = self.topology.shortest_path(old_node, new_node)
            self.previous_class = dominant_class
        shimmer = 0.025 * feature.rms * np.sin(np.arange(self.topology.count) * 1.7 + frame.monotonic_time * 2.2)
        self.state.apex_scale[:] = 1.0 + 0.56 * self.energy + shimmer
        self.state.radial_offset[:] = self.topology.core_radius * 0.09 * self.energy
        self.state.uniform_scale[:] = 1.0
        self.state.line_alpha[:] = 0.61 + 0.37 * self.energy
        self.state.line_width[:] = 1.35 + 1.7 * self.energy
        self.state.accent_mix[:] = np.clip(0.72 * self.energy, 0.0, 0.82)
        self.state.accent_hue[:] = np.mod((self.topology.azimuth + math.pi) / (2.0 * math.pi), 1.0)
        self.state.particle_rate[:] = self.energy * (4.0 if feature.onset and coherence > 0.22 else 0.0)
        self.state.route = self.route
        self.state.route_energy = float(np.clip(coherence * feature.rms * 1.5, 0.0, 1.0))
        return self.state
