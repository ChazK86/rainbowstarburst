"""Signal Relay: fixed-step damped waves over the exact pyramid graph."""

import numpy as np

from .base import AudioAnimation, combine_features


class SignalRelay(AudioAnimation):
    name = "RELAY"

    def __init__(self, topology):
        super().__init__(topology)
        self.displacement = np.zeros(topology.count, dtype=float)
        self.velocity = np.zeros(topology.count, dtype=float)
        self.accumulator = 0.0
        self.fixed_dt = 1.0 / 120.0
        self.coupling = 34.0
        self.damping = 8.0

    def reset(self):
        super().reset()
        self.displacement[:] = 0.0
        self.velocity[:] = 0.0
        self.accumulator = 0.0

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        del now
        feature = combine_features(frame)
        injection = np.zeros(self.topology.count, dtype=float)
        if self.topology.count:
            for band, seed in enumerate(self.topology.band_seeds):
                injection[seed] += float(feature.bands[band]) * gain * (8.0 + 8.0 * feature.flux)
        if self.is_new_frame(frame) and feature.onset and self.topology.count:
            strongest = int(np.argmax(feature.bands))
            self.velocity[self.topology.band_seeds[strongest]] += 1.1 + 2.0 * feature.flux
        self.accumulator += min(float(dt), 0.1)
        steps = 0
        while self.accumulator + 1e-12 >= self.fixed_dt and steps < 12:
            acceleration = -self.damping * self.velocity + injection
            if len(self.topology.edges):
                first = self.topology.edges[:, 0]
                second = self.topology.edges[:, 1]
                difference = self.displacement[second] - self.displacement[first]
                np.add.at(acceleration, first, self.coupling * difference)
                np.add.at(acceleration, second, -self.coupling * difference)
            self.velocity += acceleration * self.fixed_dt
            self.displacement += self.velocity * self.fixed_dt
            self.displacement[:] = np.clip(self.displacement, -1.2, 1.2)
            self.velocity[:] = np.clip(self.velocity, -8.0, 8.0)
            self.accumulator -= self.fixed_dt
            steps += 1
        if steps == 12:
            self.accumulator = min(self.accumulator, self.fixed_dt)
        absolute = np.abs(self.displacement)
        normalized = np.clip(absolute / 0.65, 0.0, 1.0)
        self.state.apex_scale[:] = 1.0 + np.clip(self.displacement * 0.62, -0.42, 0.82)
        self.state.radial_offset[:] = self.topology.core_radius * np.clip(self.displacement * 0.12, -0.22, 0.22)
        self.state.uniform_scale[:] = 1.0
        self.state.line_alpha[:] = 0.6 + 0.4 * normalized
        self.state.line_width[:] = 1.35 + 2.0 * normalized
        self.state.accent_mix[:] = 0.78 * normalized
        self.state.accent_hue[:] = np.mod(0.58 + self.displacement * 0.35, 1.0)
        collision = np.maximum(0.0, normalized - 0.82)
        self.state.particle_rate[:] = collision * (2.0 + 6.0 * feature.flux)
        start = int(np.argmax(absolute)) if self.topology.count else 0
        self.state.route = self.topology.greedy_route(self.displacement, start=start, limit=64)
        self.state.route_energy = float(normalized.max(initial=0.0))
        return self.state
