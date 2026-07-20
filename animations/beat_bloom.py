"""Beat Bloom: kick bloom, snare graph ring, and hat parity shimmer."""

import math

import numpy as np

from .base import AudioAnimation, combine_features


class BeatBloom(AudioAnimation):
    name = "BLOOM"

    def __init__(self, topology):
        super().__init__(topology)
        self.kick = 0.0
        self.hat = 0.0
        self.ring_age = 99.0
        self.ring_seed = 0

    def reset(self):
        super().reset()
        self.kick = self.hat = 0.0
        self.ring_age = 99.0

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        feature = combine_features(frame)
        self.kick *= math.exp(-dt / (0.24 / reactivity))
        self.hat *= math.exp(-dt / (0.075 / reactivity))
        self.ring_age += dt
        if self.is_new_frame(frame) and feature.onset:
            low = float(np.mean(feature.bands[:2])) * (0.5 + 0.5 * feature.low_flux)
            high = float(np.mean(feature.bands[6:])) * (0.4 + 0.6 * feature.high_flux)
            middle = float(np.mean(feature.bands[2:6]))
            if low >= max(high * 0.7, middle * 0.55):
                self.kick = max(self.kick, np.clip(low * gain * 1.5, 0.0, 1.0))
            if middle + high * 0.65 > low * 0.8:
                self.ring_age = 0.0
                self.ring_seed = (frame.sequence * 17) % max(1, self.topology.count)
            if high > 0.12:
                self.hat = max(self.hat, np.clip(high * gain * 1.8, 0.0, 1.0))

        if self.topology.count:
            distance = self.topology.distances_from(self.ring_seed).astype(float)
            speed = 28.0 * reactivity
            ring_position = self.ring_age * speed
            ring = np.exp(-0.5 * np.square((distance - ring_position) / 0.8))
        else:
            distance = ring = np.zeros(0)
        parity = (self.topology.parity == int((now * 22.0) % 2)).astype(float)
        breath = 0.025 * feature.tempo_confidence * math.sin(feature.beat_phase * 2.0 * math.pi)
        self.state.uniform_scale[:] = 1.0 + breath + 0.055 * self.kick
        self.state.apex_scale[:] = 1.0 + 0.55 * self.kick + 0.46 * ring + 0.18 * self.hat * parity
        self.state.radial_offset[:] = self.topology.core_radius * (0.035 * self.kick + 0.08 * ring)
        self.state.line_alpha[:] = 0.64 + 0.3 * np.maximum(ring, self.kick)
        self.state.line_width[:] = 1.4 + 1.8 * np.maximum(ring, self.hat * parity)
        self.state.accent_mix[:] = np.clip(0.78 * ring + 0.35 * self.hat * parity, 0.0, 0.85)
        self.state.accent_hue[:] = np.mod(distance / max(1.0, distance.max(initial=1.0)) + feature.beat_phase, 1.0)
        self.state.particle_rate[:] = ring * max(0.0, feature.flux - 0.5) * 12.0
        self.state.route = tuple(np.argsort(distance).tolist()) if len(distance) else ()
        self.state.route_energy = float(np.clip(max(self.kick, self.hat, feature.flux), 0.0, 1.0))
        return self.state
