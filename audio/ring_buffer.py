"""A small bounded audio ring buffer shared by capture and analysis threads."""

from __future__ import annotations

import threading

import numpy as np


class AudioRingBuffer:
    """Preallocated, bounded FIFO for interleaved floating-point audio frames.

    The capture callback never waits for the analyzer.  When the analyzer falls
    behind, the oldest frames are discarded and ``dropped_frames`` records the
    loss.  A short lock protects NumPy copies because callbacks and analysis run
    on separate native threads.
    """

    def __init__(self, capacity_frames: int, channels: int):
        if capacity_frames < 2:
            raise ValueError("capacity_frames must be at least 2")
        if channels < 1:
            raise ValueError("channels must be positive")
        self.capacity = int(capacity_frames)
        self.channels = int(channels)
        self._data = np.zeros((self.capacity, self.channels), dtype=np.float32)
        self._read = 0
        self._write = 0
        self._size = 0
        self._dropped_frames = 0
        self._lock = threading.Lock()

    @property
    def available(self) -> int:
        with self._lock:
            return self._size

    @property
    def dropped_frames(self) -> int:
        with self._lock:
            return self._dropped_frames

    def clear(self) -> None:
        with self._lock:
            self._read = 0
            self._write = 0
            self._size = 0

    def write(self, frames: np.ndarray) -> None:
        array = np.asarray(frames, dtype=np.float32)
        if array.ndim == 1:
            array = array.reshape(-1, self.channels)
        if array.ndim != 2 or array.shape[1] != self.channels:
            raise ValueError(f"expected frames shaped (n, {self.channels})")
        count = int(array.shape[0])
        if count == 0:
            return

        with self._lock:
            if count >= self.capacity:
                discarded = self._size + count - self.capacity
                array = array[-self.capacity :]
                count = self.capacity
                self._read = 0
                self._write = 0
                self._size = 0
                self._dropped_frames += max(0, discarded)

            overflow = max(0, self._size + count - self.capacity)
            if overflow:
                self._read = (self._read + overflow) % self.capacity
                self._size -= overflow
                self._dropped_frames += overflow

            first = min(count, self.capacity - self._write)
            self._data[self._write : self._write + first] = array[:first]
            remaining = count - first
            if remaining:
                self._data[:remaining] = array[first:]
            self._write = (self._write + count) % self.capacity
            self._size += count

    def read(self, count: int) -> np.ndarray:
        requested = max(0, int(count))
        with self._lock:
            count = min(requested, self._size)
            result = np.empty((count, self.channels), dtype=np.float32)
            if count == 0:
                return result
            first = min(count, self.capacity - self._read)
            result[:first] = self._data[self._read : self._read + first]
            remaining = count - first
            if remaining:
                result[first:] = self._data[:remaining]
            self._read = (self._read + count) % self.capacity
            self._size -= count
            return result
