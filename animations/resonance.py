"""Per-pyramid acoustic identities and organic spring micro-dynamics."""

from __future__ import annotations

import math

import numpy as np

from .base import RenderStateBatch, combine_features


class OrganicResonatorBank:
    """Turn the fine spectrum into one coupled physical voice per pyramid.

    Each node has a unique center frequency, bandwidth, harmonic mixture,
    natural response rate, damping ratio, and modulation phase.  The simulation
    uses a fixed step so its motion does not change with render frame rate.
    """

    def __init__(self, topology):
        self.topology = topology
        self.displacement = np.zeros(topology.count, dtype=float)
        self.velocity = np.zeros(topology.count, dtype=float)
        self.drive = np.zeros(topology.count, dtype=float)
        self.state = RenderStateBatch.identity(topology.count)
        variation = topology.resonance_variation
        natural_hz = 0.72 + 1.65 * variation
        self.stiffness = np.square(2.0 * math.pi * natural_hz)
        damping_ratio = 0.52 + 0.34 * np.mod(variation * 1.61803398875, 1.0)
        self.damping = 2.0 * damping_ratio * np.sqrt(self.stiffness)
        self.phase = np.mod(variation * 17.0 + np.arange(topology.count) * 0.73, 2.0 * math.pi)
        self.motion_hz = 0.45 + 1.35 * np.mod(variation * 2.41421356237, 1.0)
        self.fixed_dt = 1.0 / 120.0
        self.accumulator = 0.0
        self.last_sequence = -1
        self.coupling = 4.2

    def reset(self):
        self.displacement[:] = 0.0
        self.velocity[:] = 0.0
        self.drive[:] = 0.0
        self.state = RenderStateBatch.identity(self.topology.count)
        self.accumulator = 0.0
        self.last_sequence = -1

    def update(self, dt, now, frame, gain=1.0):
        feature = combine_features(frame)
        spectrum = np.asarray(feature.spectrum, dtype=float)
        if self.topology.count and len(spectrum):
            acoustic = self.topology.resonance_weights @ spectrum
            personality = 0.78 + 0.44 * self.topology.resonance_variation
            flutter = 1.0 + 0.055 * np.sin(
                self.phase + now * 2.0 * math.pi * self.motion_hz
            )
            target = np.clip(acoustic * personality * flutter * gain * 3.2, 0.0, 1.0)
            self.drive[:] = np.power(target, 1.12)
        else:
            self.drive[:] = 0.0

        if frame.sequence != self.last_sequence:
            self.last_sequence = frame.sequence
            if feature.onset:
                self.velocity += self.drive * (0.4 + 1.2 * feature.flux)

        self.accumulator += min(max(0.0, float(dt)), 0.1)
        steps = 0
        while self.accumulator + 1e-12 >= self.fixed_dt and steps < 12:
            acceleration = (
                self.stiffness * (self.drive - self.displacement)
                - self.damping * self.velocity
            )
            if len(self.topology.edges):
                first = self.topology.edges[:, 0]
                second = self.topology.edges[:, 1]
                difference = self.displacement[second] - self.displacement[first]
                np.add.at(acceleration, first, self.coupling * difference)
                np.add.at(acceleration, second, -self.coupling * difference)
            self.velocity += acceleration * self.fixed_dt
            self.displacement += self.velocity * self.fixed_dt
            self.displacement[:] = np.clip(self.displacement, -0.32, 1.18)
            self.velocity[:] = np.clip(self.velocity, -9.0, 9.0)
            self.accumulator -= self.fixed_dt
            steps += 1
        if steps == 12:
            self.accumulator = min(self.accumulator, self.fixed_dt)

        positive = np.clip(self.displacement, 0.0, 1.0)
        magnitude = np.clip(np.abs(self.displacement), 0.0, 1.0)
        self.state.apex_scale[:] = 1.0 + 0.24 * self.displacement
        self.state.radial_offset[:] = self.topology.core_radius * 0.035 * self.displacement
        self.state.uniform_scale[:] = 1.0
        self.state.rotation_offset[:] = 0.0
        self.state.line_alpha[:] = 0.68 + 0.16 * magnitude
        self.state.line_width[:] = 1.5 + 0.58 * magnitude
        # Keep the authored wireframe recognizably white; frequency color is a
        # subtle active tint while the traveling route carries full rainbow.
        self.state.accent_mix[:] = 0.12 * positive
        log_frequency = np.log2(
            self.topology.resonant_frequencies / max(1e-9, self.topology.resonant_frequencies.min(initial=35.0))
        )
        hue_span = max(float(log_frequency.max(initial=1.0)), 1e-9)
        self.state.accent_hue[:] = log_frequency / hue_span
        self.state.particle_rate[:] = 0.0
        self.state.route = self.topology.greedy_route(magnitude, limit=64)
        self.state.route_energy = float(np.clip(magnitude.max(initial=0.0), 0.0, 1.0))
        return self.state
