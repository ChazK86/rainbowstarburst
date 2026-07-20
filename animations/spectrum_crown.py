"""Spectrum Crown: distribute eight frequency bands over precise face clusters."""

import numpy as np

from .base import AudioAnimation, combine_features, smooth


class SpectrumCrown(AudioAnimation):
    name = "CROWN"

    def __init__(self, topology):
        super().__init__(topology)
        self.energy = np.zeros(topology.count, dtype=float)
        self.spark = np.zeros(topology.count, dtype=float)

    def reset(self):
        super().reset()
        self.energy[:] = 0.0
        self.spark[:] = 0.0

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        del now
        feature = combine_features(frame)
        target = self.topology.band_influence @ np.clip(feature.bands * gain, 0.0, 1.0)
        target /= np.maximum(self.topology.band_influence.sum(axis=1), 0.45)
        target = np.clip(target * 2.2, 0.0, 1.0)
        smooth(self.energy, target, dt, attack=0.025 / reactivity, release=0.34 / reactivity)
        self.state.apex_scale[:] = 1.0 + 0.72 * np.power(self.energy, 1.35)
        self.state.radial_offset[:] = 0.16 * self.topology.core_radius * self.energy
        self.state.uniform_scale[:] = 1.0
        self.state.line_alpha[:] = 0.62 + 0.38 * self.energy
        self.state.line_width[:] = 1.35 + 1.65 * self.energy
        self.state.accent_mix[:] = 0.72 * self.energy
        primary_band = np.argmax(self.topology.band_influence, axis=1)
        self.state.accent_hue[:] = np.mod(feature.centroid * 0.45 + primary_band / 8.0, 1.0)
        self.spark *= np.exp(-dt / 0.16)
        if self.is_new_frame(frame) and feature.onset:
            strongest_band = int(np.argmax(feature.bands))
            cluster = self.topology.band_influence[:, strongest_band]
            self.spark += cluster * (2.0 + 10.0 * feature.flux)
        self.state.particle_rate[:] = self.spark
        self.state.route = self.topology.greedy_route(self.energy)
        self.state.route_energy = float(max(feature.rms, feature.flux * 0.8))
        return self.state
