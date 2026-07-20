"""Lifecycle, crossfades, bounds, and formation changes for all programs."""

from __future__ import annotations

import numpy as np

from audio.features import AudioFeatureFrame

from .base import RenderStateBatch
from .beat_bloom import BeatBloom
from .cinematic_weather import CinematicWeather
from .conversation_orbit import ConversationOrbit
from .game_radar import GameRadar
from .harmonic_aurora import HarmonicAurora
from .resonance import OrganicResonatorBank
from .signal_relay import SignalRelay
from .spectrum_crown import SpectrumCrown
from .topology import FormationTopology


PROGRAM_TYPES = {
    "CROWN": SpectrumCrown,
    "BLOOM": BeatBloom,
    "ORBIT": ConversationOrbit,
    "CINEMA": CinematicWeather,
    "RADAR": GameRadar,
    "RELAY": SignalRelay,
    "AURORA": HarmonicAurora,
}
PROGRAM_NAMES = tuple(PROGRAM_TYPES)


class AnimationDirector:
    def __init__(self, pyramids=()):
        self.topology = FormationTopology(pyramids)
        self.programs = {name: program_type(self.topology) for name, program_type in PROGRAM_TYPES.items()}
        self.resonators = OrganicResonatorBank(self.topology)
        self.active_name: str | None = None
        self.output = RenderStateBatch.identity(self.topology.count)
        self._transition_from = self.output.copy()
        self._transition_elapsed = 0.25
        self.transition_seconds = 0.25
        self.sensitivity = 1.0
        self.reactivity = 1.0
        self.reduced_motion = False
        self._silence_elapsed = 0.0

    @property
    def active(self) -> bool:
        return self.active_name is not None

    def set_formation(self, pyramids) -> None:
        selected = self.active_name
        self.topology = FormationTopology(pyramids)
        self.programs = {name: program_type(self.topology) for name, program_type in PROGRAM_TYPES.items()}
        self.resonators = OrganicResonatorBank(self.topology)
        self.output = RenderStateBatch.identity(self.topology.count)
        self._transition_from = self.output.copy()
        self._transition_elapsed = self.transition_seconds
        self.active_name = selected if selected in self.programs else None
        self._silence_elapsed = 0.0

    def select(self, name: str | None) -> None:
        normalized = None if not name or str(name).upper() in ("NONE", "OFF") else str(name).upper()
        if normalized is not None and normalized not in self.programs:
            raise ValueError(f"unknown audio program: {name}")
        if normalized == self.active_name:
            return
        self._transition_from = self.output.copy()
        self._transition_elapsed = 0.0
        self.active_name = normalized
        self._silence_elapsed = 0.0
        if normalized is not None:
            self.programs[normalized].reset()
        else:
            self.resonators.reset()
            self.output = RenderStateBatch.identity(self.topology.count)
            self._transition_from = self.output.copy()
            self._transition_elapsed = self.transition_seconds

    def update(self, dt: float, now: float, frame: AudioFeatureFrame | None = None) -> RenderStateBatch:
        if not self.active_name:
            self.output = RenderStateBatch.identity(self.topology.count)
            return self.output
        frame = frame or AudioFeatureFrame.silence()
        if (
            not frame.system.active
            and not frame.microphone.active
            and max(frame.system.rms, frame.microphone.rms) < 0.01
        ):
            self._silence_elapsed += max(0.0, dt)
        else:
            self._silence_elapsed = 0.0
        if self._silence_elapsed >= 1.5 - 1e-9:
            self.programs[self.active_name].reset()
            self.resonators.reset()
            self.output = RenderStateBatch.identity(self.topology.count)
            self._transition_from = self.output.copy()
            self._transition_elapsed = self.transition_seconds
            return self.output
        program_target = self.programs[self.active_name].update(
            min(max(0.0, float(dt)), 0.1),
            float(now),
            frame,
            gain=self.sensitivity,
            reactivity=self.reactivity,
        )
        resonance = self.resonators.update(
            min(max(0.0, float(dt)), 0.1),
            float(now),
            frame,
            gain=self.sensitivity,
        )
        target = program_target.copy()
        target.apex_scale += resonance.apex_scale - 1.0
        target.radial_offset += resonance.radial_offset
        target.line_alpha[:] = np.maximum(target.line_alpha, resonance.line_alpha)
        target.line_width += resonance.line_width - 1.5
        target.accent_mix[:] = np.clip(
            target.accent_mix + resonance.accent_mix,
            0.0,
            0.88,
        )
        # Let the unique resonator hue lead only where its acoustic displacement
        # is stronger than the program's macro response.
        resonance_strength = np.clip(np.abs(self.resonators.displacement), 0.0, 1.0)
        target.accent_hue[:] = np.mod(
            target.accent_hue * (1.0 - resonance_strength)
            + resonance.accent_hue * resonance_strength,
            1.0,
        )
        if resonance.route_energy > target.route_energy:
            target.route = resonance.route
            target.route_energy = resonance.route_energy
        if self._transition_elapsed < self.transition_seconds:
            self._transition_elapsed += max(0.0, dt)
            amount = min(1.0, self._transition_elapsed / self.transition_seconds)
            # Cubic smoothstep avoids a visible velocity discontinuity.
            amount = amount * amount * (3.0 - 2.0 * amount)
            self.output = RenderStateBatch.blend(self._transition_from, target, amount)
        else:
            self.output = target
        self.output.clamp(self.topology.core_radius, self.reduced_motion)
        return self.output
