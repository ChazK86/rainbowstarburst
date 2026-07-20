"""Adaptive level normalization and activity gating."""

from __future__ import annotations

from collections import deque
import math

import numpy as np


class AdaptiveRange:
    """Rolling robust 0..1 normalization with attack/release smoothing."""

    def __init__(
        self,
        history: int = 400,
        attack_seconds: float = 0.035,
        release_seconds: float = 0.28,
        minimum_ceiling: float = 1e-4,
    ):
        self.values = deque(maxlen=max(16, int(history)))
        self.attack_seconds = max(1e-4, float(attack_seconds))
        self.release_seconds = max(1e-4, float(release_seconds))
        self.minimum_ceiling = max(1e-12, float(minimum_ceiling))
        self.output = 0.0
        self.floor = 0.0
        self.ceiling = self.minimum_ceiling

    def update(self, value: float, dt: float) -> float:
        value = max(0.0, float(value))
        self.values.append(value)
        samples = np.fromiter(self.values, dtype=float)
        if len(samples) >= 12:
            self.floor = float(np.percentile(samples, 10.0))
            self.ceiling = float(np.percentile(samples, 95.0))
        else:
            self.floor = 0.0
            self.ceiling = max(self.minimum_ceiling, float(samples.max(initial=0.0)))
        span = max(self.minimum_ceiling, self.ceiling - self.floor)
        target = float(np.clip((value - self.floor) / span, 0.0, 1.0))
        seconds = self.attack_seconds if target > self.output else self.release_seconds
        alpha = 1.0 - math.exp(-max(0.0, dt) / seconds)
        self.output += (target - self.output) * alpha
        return float(np.clip(self.output, 0.0, 1.0))


class ActivityGate:
    """Hysteretic activity state with a short release hang time."""

    def __init__(self, open_level: float = 0.085, close_level: float = 0.045, hang=0.2):
        self.open_level = float(open_level)
        self.close_level = float(close_level)
        self.hang = float(hang)
        self.active = False
        self._quiet_for = 0.0

    def update(self, level: float, voice_ratio: float, dt: float) -> bool:
        evidence = float(level) * (0.65 + 0.35 * float(voice_ratio))
        if not self.active and evidence >= self.open_level:
            self.active = True
            self._quiet_for = 0.0
        elif self.active:
            if evidence < self.close_level:
                self._quiet_for += max(0.0, dt)
                if self._quiet_for >= self.hang:
                    self.active = False
                    self._quiet_for = 0.0
            else:
                self._quiet_for = 0.0
        return self.active
