"""Conversation Orbit: separate system and microphone activity hemispheres."""

import math

import numpy as np

from .base import AudioAnimation, smooth


class ConversationOrbit(AudioAnimation):
    name = "ORBIT"

    def __init__(self, topology):
        super().__init__(topology)
        x = topology.normals[:, 0] if topology.count else np.zeros(0)
        self.system_weight = np.clip(0.15 + 0.85 * (x + 1.0) * 0.5, 0.15, 1.0)
        self.mic_weight = np.clip(0.15 + 0.85 * (1.0 - x) * 0.5, 0.15, 1.0)
        self.energy = np.zeros(topology.count)
        self.last_active = "quiet"
        self.handoff_age = 99.0
        self.handoff_route = ()
        self.system_node = int(np.argmax(x)) if topology.count else 0
        self.mic_node = int(np.argmin(x)) if topology.count else 0

    def reset(self):
        super().reset()
        self.energy[:] = 0.0
        self.last_active = "quiet"
        self.handoff_age = 99.0
        self.handoff_route = ()

    def update(self, dt, now, frame, gain=1.0, reactivity=1.0):
        system = frame.system.rms if frame.system.active else 0.0
        microphone = frame.microphone.rms if frame.microphone.active else 0.0
        target = np.clip((system * self.system_weight + microphone * self.mic_weight) * gain, 0.0, 1.0)
        smooth(self.energy, target, dt, attack=0.07 / reactivity, release=0.42 / reactivity)
        current = "both" if system > 0.08 and microphone > 0.08 else (
            "system" if system > microphone and system > 0.08 else "microphone" if microphone > 0.08 else "quiet"
        )
        if self.is_new_frame(frame) and current in ("system", "microphone") and self.last_active in ("system", "microphone") and current != self.last_active:
            start = self.system_node if self.last_active == "system" else self.mic_node
            goal = self.system_node if current == "system" else self.mic_node
            self.handoff_route = self.topology.shortest_path(start, goal)
            self.handoff_age = 0.0
        if current != "quiet":
            self.last_active = current
        self.handoff_age += dt
        equator = np.exp(-np.square(self.topology.latitude / 0.32)) if self.topology.count else np.zeros(0)
        overlap = min(system, microphone)
        route_energy = max(0.0, 1.0 - self.handoff_age / 0.7)
        route_mask = np.zeros(self.topology.count)
        if self.handoff_route and route_energy > 0.0:
            progress = self.handoff_age / 0.7 * len(self.handoff_route)
            for index, node in enumerate(self.handoff_route):
                route_mask[node] = math.exp(-0.5 * ((index - progress) / 1.4) ** 2)
        self.state.apex_scale[:] = 1.0 + 0.55 * self.energy + 0.28 * overlap * equator
        self.state.radial_offset[:] = self.topology.core_radius * (0.08 * self.energy + 0.08 * route_mask)
        self.state.uniform_scale[:] = 1.0 + 0.035 * overlap
        self.state.rotation_offset[:, 1] = overlap * math.sin(now * 0.8) * 5.0
        self.state.line_alpha[:] = 0.62 + 0.34 * self.energy
        self.state.line_width[:] = 1.4 + 1.5 * self.energy
        self.state.accent_mix[:] = np.clip(0.45 * self.energy + 0.7 * route_mask, 0.0, 0.85)
        self.state.accent_hue[:] = np.where(self.topology.normals[:, 0] >= 0.0, 0.56, 0.03)
        self.state.particle_rate[:] = np.where((system + microphone) > 1.4, equator * 2.0, 0.0)
        self.state.route = self.handoff_route
        self.state.route_energy = float(route_energy)
        return self.state
