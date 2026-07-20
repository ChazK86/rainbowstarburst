"""Source orchestration, simulation, recovery, and immutable snapshots."""

from __future__ import annotations

from enum import Enum
import math
import threading
import time

import numpy as np

from .capture import AudioCapture, CaptureStatus
from .devices import AudioDeviceManager, BACKEND_NAME, HAS_WASAPI_LOOPBACK
from .features import AudioFeatureFrame, FeatureAnalyzer, SourceFeatures


class AudioSourceMode(str, Enum):
    OFF = "OFF"
    SYSTEM = "SYSTEM"
    MICROPHONE = "MIC"
    BOTH = "BOTH"
    DEMO = "DEMO"


class AudioEngine:
    """Own capture sessions while exposing a lock-free-sized feature snapshot."""

    def __init__(self):
        self.device_manager = AudioDeviceManager()
        self.mode = AudioSourceMode.OFF
        self.system_device_id: str | None = None
        self.microphone_device_id: str | None = None
        self._system = SourceFeatures.silence()
        self._microphone = SourceFeatures.silence()
        self._system_status = CaptureStatus(message="SYSTEM OFF")
        self._microphone_status = CaptureStatus(message="MIC OFF")
        self._system_capture: AudioCapture | None = None
        self._microphone_capture: AudioCapture | None = None
        self._sequence = 0
        self._lock = threading.RLock()
        self._demo_stop = threading.Event()
        self._demo_thread: threading.Thread | None = None
        self._monitor_stop = threading.Event()
        self._next_retry = 0.0
        self._monitor = threading.Thread(target=self._monitor_loop, name="rainbowstarburst-audio-monitor", daemon=True)
        self._monitor.start()

    @property
    def backend_name(self) -> str:
        return BACKEND_NAME

    @property
    def supports_loopback(self) -> bool:
        return HAS_WASAPI_LOOPBACK

    def devices(self):
        try:
            return self.device_manager.enumerate()
        except Exception:
            return [], []

    def configure(
        self,
        mode: AudioSourceMode | str,
        system_device_id: str | None = None,
        microphone_device_id: str | None = None,
    ) -> None:
        selected = mode if isinstance(mode, AudioSourceMode) else AudioSourceMode(str(mode).upper())
        with self._lock:
            self.mode = selected
            if system_device_id is not None:
                self.system_device_id = system_device_id
            if microphone_device_id is not None:
                self.microphone_device_id = microphone_device_id
        self._stop_sessions()
        if selected == AudioSourceMode.DEMO:
            self._start_demo()
        elif selected != AudioSourceMode.OFF:
            self._ensure_sessions(force=True)

    def snapshot(self) -> AudioFeatureFrame:
        with self._lock:
            system_status = self._system_capture.status if self._system_capture is not None else self._system_status
            microphone_status = self._microphone_capture.status if self._microphone_capture is not None else self._microphone_status
            system = self._system if self.mode in (AudioSourceMode.SYSTEM, AudioSourceMode.BOTH, AudioSourceMode.DEMO) else SourceFeatures.silence()
            microphone = self._microphone if self.mode in (AudioSourceMode.MICROPHONE, AudioSourceMode.BOTH, AudioSourceMode.DEMO) else SourceFeatures.silence()
            latency = max(system_status.latency_ms, microphone_status.latency_ms)
            dropped = system_status.dropped_windows + microphone_status.dropped_windows
            return AudioFeatureFrame(self._sequence, time.monotonic(), system, microphone, latency, dropped)

    def status(self) -> dict[str, object]:
        with self._lock:
            system_status = self._system_capture.status if self._system_capture is not None else self._system_status
            microphone_status = self._microphone_capture.status if self._microphone_capture is not None else self._microphone_status
            return {
                "mode": self.mode.value,
                "backend": BACKEND_NAME,
                "loopback_supported": HAS_WASAPI_LOOPBACK,
                "system": system_status,
                "microphone": microphone_status,
            }

    def shutdown(self) -> None:
        with self._lock:
            self.mode = AudioSourceMode.OFF
        self._monitor_stop.set()
        self._stop_sessions()
        if self._monitor is not threading.current_thread():
            self._monitor.join(timeout=1.0)

    def _publish_system(self, feature: SourceFeatures, status: CaptureStatus) -> None:
        with self._lock:
            self._system = feature
            self._system_status = status
            self._sequence += 1

    def _publish_microphone(self, feature: SourceFeatures, status: CaptureStatus) -> None:
        with self._lock:
            self._microphone = feature
            self._microphone_status = status
            self._sequence += 1

    def _stop_sessions(self) -> None:
        self._demo_stop.set()
        demo, self._demo_thread = self._demo_thread, None
        if demo is not None and demo is not threading.current_thread():
            demo.join(timeout=1.0)
        for capture_name in ("_system_capture", "_microphone_capture"):
            capture = getattr(self, capture_name)
            setattr(self, capture_name, None)
            if capture is not None:
                capture.stop()
        with self._lock:
            self._system = SourceFeatures.silence()
            self._microphone = SourceFeatures.silence()
            if self.mode == AudioSourceMode.OFF:
                self._system_status = CaptureStatus(message="SYSTEM OFF")
                self._microphone_status = CaptureStatus(message="MIC OFF")

    def _ensure_sessions(self, force=False) -> None:
        with self._lock:
            mode = self.mode
        if mode in (AudioSourceMode.OFF, AudioSourceMode.DEMO):
            return
        now = time.monotonic()
        if not force and now < self._next_retry:
            return
        self._next_retry = now + 3.0
        wants_system = mode in (AudioSourceMode.SYSTEM, AudioSourceMode.BOTH)
        wants_mic = mode in (AudioSourceMode.MICROPHONE, AudioSourceMode.BOTH)
        if wants_system and (self._system_capture is None or not self._system_capture.is_active):
            if self._system_capture is not None:
                self._system_capture.stop()
            self._system_capture = self._open_capture("system", self.system_device_id, self._publish_system)
        if wants_mic and (self._microphone_capture is None or not self._microphone_capture.is_active):
            if self._microphone_capture is not None:
                self._microphone_capture.stop()
            self._microphone_capture = self._open_capture("microphone", self.microphone_device_id, self._publish_microphone)

    def _open_capture(self, kind, identifier, publish) -> AudioCapture | None:
        try:
            device = self.device_manager.resolve(kind, identifier)
            if device is None:
                raise RuntimeError("no compatible endpoint found")
            capture = AudioCapture(device, publish)
            capture.start()
            if kind == "system":
                self.system_device_id = device.identifier
                self._system_status = capture.status
            else:
                self.microphone_device_id = device.identifier
                self._microphone_status = capture.status
            return capture
        except Exception as exc:
            status = CaptureStatus("unavailable", f"{kind.upper()} UNAVAILABLE: {exc}")
            with self._lock:
                if kind == "system":
                    self._system_status = status
                    self._system = SourceFeatures.silence()
                else:
                    self._microphone_status = status
                    self._microphone = SourceFeatures.silence()
            return None

    def _monitor_loop(self) -> None:
        while not self._monitor_stop.wait(1.0):
            self._ensure_sessions()

    def _start_demo(self) -> None:
        self._demo_stop.clear()
        self._demo_thread = threading.Thread(target=self._demo_loop, name="rainbowstarburst-demo-audio", daemon=True)
        self._demo_thread.start()

    def _demo_loop(self) -> None:
        rate, hop = 48000, 512
        system_analyzer = FeatureAnalyzer(rate, 2)
        mic_analyzer = FeatureAnalyzer(rate, 1)
        cursor = 0
        start = time.monotonic()
        while not self._demo_stop.is_set():
            t = (np.arange(hop, dtype=float) + cursor) / rate
            elapsed = time.monotonic() - start
            beat = (elapsed % 0.5) < (hop / rate * 1.5)
            kick = (0.7 * np.sin(2.0 * math.pi * 58.0 * t) * np.exp(-((elapsed % 0.5) * 18.0)))
            sweep_frequency = 90.0 * (2.0 ** ((elapsed % 8.0) / 8.0 * 7.0))
            music = 0.17 * np.sin(2.0 * math.pi * sweep_frequency * t) + kick * float(beat)
            pan = math.sin(elapsed * 0.7)
            system_chunk = np.column_stack((music * (1.0 - 0.55 * pan), music * (1.0 + 0.55 * pan))).astype(np.float32)
            talk_gate = 1.0 if 2.5 < (elapsed % 6.0) < 4.7 else 0.0
            voice = talk_gate * 0.22 * (
                np.sin(2.0 * math.pi * 180.0 * t) + 0.4 * np.sin(2.0 * math.pi * 540.0 * t)
            )
            mic_chunk = voice[:, None].astype(np.float32)
            now = time.monotonic()
            system = system_analyzer.push(system_chunk, now)
            microphone = mic_analyzer.push(mic_chunk, now)
            status = CaptureStatus("demo", "DEMO SIGNAL", 21.3, 0)
            if system is not None:
                self._publish_system(system, status)
            if microphone is not None:
                self._publish_microphone(microphone, status)
            cursor += hop
            self._demo_stop.wait(hop / rate)
