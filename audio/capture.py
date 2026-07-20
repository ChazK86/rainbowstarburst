"""PortAudio capture lifecycle and analyzer worker."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable

import numpy as np

from .devices import AudioDevice, pyaudio
from .features import FeatureAnalyzer, SourceFeatures
from .ring_buffer import AudioRingBuffer


@dataclass(frozen=True)
class CaptureStatus:
    state: str = "stopped"
    message: str = "STOPPED"
    latency_ms: float = 0.0
    dropped_windows: int = 0


class AudioCapture:
    """One selected endpoint, one callback ring, and one analyzer worker."""

    def __init__(self, device: AudioDevice, publish: Callable[[SourceFeatures, CaptureStatus], None]):
        self.device = device
        self.publish = publish
        self.ring: AudioRingBuffer | None = None
        self.analyzer: FeatureAnalyzer | None = None
        self._audio = None
        self._stream = None
        self._worker: threading.Thread | None = None
        self._stop = threading.Event()
        self._status = CaptureStatus()
        self._status_lock = threading.Lock()
        self._last_callback = 0.0

    @property
    def status(self) -> CaptureStatus:
        with self._status_lock:
            return self._status

    @property
    def is_active(self) -> bool:
        try:
            return self._stream is not None and bool(self._stream.is_active()) and not self._stop.is_set()
        except Exception:
            return False

    def start(self) -> None:
        if pyaudio is None:
            raise RuntimeError("PyAudio is not installed")
        self.stop()
        self._stop.clear()
        self._audio = pyaudio.PyAudio()
        rate = self.device.sample_rate
        channels = self.device.channels
        self.ring = AudioRingBuffer(max(rate * 2, 4096), channels)
        self.analyzer = FeatureAnalyzer(rate, channels)

        def callback(in_data, frame_count, time_info, status_flags):
            del time_info, status_flags
            if self._stop.is_set():
                return (None, pyaudio.paComplete)
            try:
                samples = np.frombuffer(in_data, dtype=np.float32)
                usable = (samples.size // channels) * channels
                if usable:
                    self.ring.write(samples[:usable].reshape(-1, channels))
                    self._last_callback = time.monotonic()
            except Exception as exc:
                self._set_status("error", f"CALLBACK ERROR: {exc}")
            return (None, pyaudio.paContinue)

        try:
            self._stream = self._audio.open(
                format=pyaudio.paFloat32,
                channels=channels,
                rate=rate,
                input=True,
                input_device_index=self.device.index,
                frames_per_buffer=512,
                stream_callback=callback,
                start=True,
            )
        except Exception:
            self.stop()
            raise

        self._set_status("live", f"LIVE: {self.device.name}", 1024 / rate * 1000.0, 0)
        self._worker = threading.Thread(
            target=self._analysis_loop,
            name=f"rainbowstarburst-{self.device.kind}-analysis",
            daemon=True,
        )
        self._worker.start()

    def stop(self) -> None:
        self._stop.set()
        stream, self._stream = self._stream, None
        if stream is not None:
            try:
                stream.stop_stream()
            except Exception:
                pass
            try:
                stream.close()
            except Exception:
                pass
        worker, self._worker = self._worker, None
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1.0)
        audio, self._audio = self._audio, None
        if audio is not None:
            try:
                audio.terminate()
            except Exception:
                pass
        self._set_status("stopped", "STOPPED")

    def _analysis_loop(self) -> None:
        assert self.ring is not None and self.analyzer is not None
        hop = self.analyzer.hop_size
        period = hop / float(self.device.sample_rate)
        while not self._stop.wait(min(0.01, period)):
            processed = False
            while self.ring.available >= hop:
                chunk = self.ring.read(hop)
                feature = self.analyzer.push(chunk, time.monotonic())
                if feature is None:
                    continue
                processed = True
                latency = (self.ring.available + self.analyzer.window_size) / self.device.sample_rate * 1000.0
                dropped_windows = self.ring.dropped_frames // hop
                status = CaptureStatus("live", f"LIVE: {self.device.name}", latency, dropped_windows)
                self._set_status(status.state, status.message, latency, dropped_windows)
                self.publish(feature, status)
            if not processed and self._last_callback and time.monotonic() - self._last_callback > 2.5:
                # Event-driven loopback endpoints can legitimately stop
                # delivering callbacks while the output mix is silent.
                self._set_status("silent", f"LIVE / SILENT: {self.device.name}")

    def _set_status(self, state, message, latency_ms=0.0, dropped_windows=0) -> None:
        with self._status_lock:
            self._status = CaptureStatus(state, str(message), float(latency_ms), int(dropped_windows))
