"""Cinematic Weather: multiscale atmosphere, presence, swell, and impact rings."""

import math

import numpy as np

from .base import AudioAnimation, combine_features, smooth


class CinematicWeather(AudioAnimation):
    name = "CINEMA"

    def __init__(self, topology):
        super().__init__(topology)
        self.short = 0.0
        self.long = 0.0
        self.previous_long = 0.0
        self.swell = 0.0
        self.impact_age = 99.0
        self.impact_seed = 0
        self.energy = np.zeros(topology.count)

    def reset(self):
        super().reset()
        self.short = self.long = self.previous_long = self.swell = 0.0
        self.impact_age = 99.0
        self.energy[:] = 0.0

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        feature = combine_features(frame)
        short_alpha = 1.0 - math.exp(-dt / 0.16)
        long_seconds = 1.5 if feature.rms >= self.long else 0.42
        long_alpha = 1.0 - math.exp(-dt / long_seconds)
        self.short += (feature.rms - self.short) * short_alpha
        self.previous_long = self.long
        self.long += (feature.rms - self.long) * long_alpha
        slope = (self.long - self.previous_long) / max(dt, 1e-5)
        self.swell += (max(0.0, slope * 2.5) - self.swell) * (1.0 - math.exp(-dt / 1.2))
        self.impact_age += dt
        if self.is_new_frame(frame) and feature.onset and feature.low_flux > 0.45 and self.impact_age > 0.32:
            target_azimuth = feature.stereo_balance * math.pi * 0.5
            target = np.array([math.cos(target_azimuth), -0.45, math.sin(target_azimuth)])
            target /= np.linalg.norm(target)
            self.impact_seed = int(np.argmax(self.topology.normals @ target)) if self.topology.count else 0
            self.impact_age = 0.0
        distance = self.topology.distances_from(self.impact_seed).astype(float) if self.topology.count else np.zeros(0)
        ring_position = self.impact_age * 13.0 * reactivity
        impact = np.exp(-0.5 * np.square((distance - ring_position) / 1.25)) * max(0.0, 1.0 - self.impact_age / 1.4)
        latitude_target = (feature.centroid - 0.5) * 1.3
        weather = np.exp(-0.5 * np.square((self.topology.latitude - latitude_target) / 0.55)) if self.topology.count else np.zeros(0)
        presence = float(np.mean(feature.bands[2:6]))
        equator = np.exp(-np.square(self.topology.latitude / 0.34)) if self.topology.count else np.zeros(0)
        target_energy = np.clip((self.long * 0.55 * weather + presence * 0.7 * equator + impact) * gain, 0.0, 1.0)
        smooth(self.energy, target_energy, dt, attack=0.12 / reactivity, release=0.48 / reactivity)
        breath = math.sin(now * 0.7) * 0.06 * self.long
        self.state.uniform_scale[:] = 1.0 + breath + 0.04 * self.swell
        self.state.apex_scale[:] = 1.0 + 0.42 * self.energy + 0.48 * impact
        self.state.radial_offset[:] = self.topology.core_radius * (0.055 * self.energy + 0.13 * impact)
        self.state.line_alpha[:] = 0.6 + 0.37 * self.energy
        self.state.line_width[:] = 1.35 + 1.8 * self.energy
        self.state.accent_mix[:] = np.clip(0.34 * weather * self.long + 0.78 * impact, 0.0, 0.84)
        self.state.accent_hue[:] = np.mod((self.topology.azimuth + math.pi) / (2.0 * math.pi) + feature.centroid * 0.3, 1.0)
        self.state.particle_rate[:] = impact * feature.low_flux * 10.0
        self.state.route = tuple(np.argsort(distance).tolist()) if len(distance) else ()
        self.state.route_energy = float(np.clip(max(self.long * 0.6, impact.max(initial=0.0)), 0.0, 1.0))
        return self.state
