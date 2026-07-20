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
        macro = self.topology.band_influence @ np.clip(feature.bands * gain, 0.0, 1.0)
        macro /= np.maximum(self.topology.band_influence.sum(axis=1), 0.45)
        micro = self.topology.resonance_weights @ np.asarray(feature.spectrum)
        target = np.clip(macro * 0.55 + micro * gain * 2.7, 0.0, 1.0)
        smooth(self.energy, target, dt, attack=0.025 / reactivity, release=0.34 / reactivity)
        self.state.apex_scale[:] = 1.0 + 0.72 * np.power(self.energy, 1.35)
        self.state.radial_offset[:] = 0.16 * self.topology.core_radius * self.energy
        self.state.uniform_scale[:] = 1.0
        self.state.line_alpha[:] = 0.62 + 0.38 * self.energy
        self.state.line_width[:] = 1.35 + 1.65 * self.energy
        self.state.accent_mix[:] = 0.72 * self.energy
        frequency_fraction = np.log2(self.topology.resonant_frequencies / 35.0) / np.log2(16000.0 / 35.0)
        self.state.accent_hue[:] = np.mod(feature.centroid * 0.25 + frequency_fraction, 1.0)
        self.spark *= np.exp(-dt / 0.16)
        if self.is_new_frame(frame) and feature.onset:
            cluster = np.clip(micro * 4.0, 0.0, 1.0)
            self.spark += cluster * (2.0 + 10.0 * feature.flux)
        self.state.particle_rate[:] = self.spark
        self.state.route = self.topology.greedy_route(self.energy)
        self.state.route_energy = float(max(feature.rms, feature.flux * 0.8))
        return self.state
