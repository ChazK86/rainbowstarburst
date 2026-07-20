"""Audio endpoint discovery with optional WASAPI loopback support."""

from __future__ import annotations

from dataclasses import dataclass
import platform


try:  # PyAudioWPatch must win because it adds loopback endpoint helpers.
    import pyaudiowpatch as pyaudio  # type: ignore

    HAS_WASAPI_LOOPBACK = platform.system() == "Windows"
    BACKEND_NAME = "PyAudioWPatch"
except ImportError:  # Standard PyAudio still provides microphone support.
    try:
        import pyaudio  # type: ignore

        HAS_WASAPI_LOOPBACK = False
        BACKEND_NAME = "PyAudio"
    except ImportError:
        pyaudio = None
        HAS_WASAPI_LOOPBACK = False
        BACKEND_NAME = "unavailable"


@dataclass(frozen=True)
class AudioDevice:
    identifier: str
    index: int
    name: str
    kind: str
    sample_rate: int
    channels: int
    is_default: bool = False


class AudioDeviceManager:
    """Enumerate microphone and loopback devices without holding them open."""

    def enumerate(self) -> tuple[list[AudioDevice], list[AudioDevice]]:
        if pyaudio is None:
            return [], []
        audio = pyaudio.PyAudio()
        try:
            default_input = self._default_index(audio, input_device=True)
            microphones = []
            loopbacks = []
            for index in range(int(audio.get_device_count())):
                try:
                    info = audio.get_device_info_by_index(index)
                except Exception:
                    continue
                channels = int(info.get("maxInputChannels", 0) or 0)
                if channels <= 0:
                    continue
                name = str(info.get("name", f"Device {index}"))
                rate = int(float(info.get("defaultSampleRate", 48000) or 48000))
                is_loopback = bool(info.get("isLoopbackDevice", False))
                kind = "system" if is_loopback else "microphone"
                device = AudioDevice(
                    identifier=f"{kind}:{index}",
                    index=index,
                    name=name,
                    kind=kind,
                    sample_rate=rate,
                    channels=min(2, channels),
                    is_default=(index == default_input and not is_loopback),
                )
                (loopbacks if is_loopback else microphones).append(device)

            default_loopback_index = self._default_loopback_index(audio, loopbacks)
            if default_loopback_index is not None:
                loopbacks = [
                    AudioDevice(**{**device.__dict__, "is_default": device.index == default_loopback_index})
                    for device in loopbacks
                ]
            microphones.sort(key=lambda item: (not item.is_default, item.name.casefold()))
            loopbacks.sort(key=lambda item: (not item.is_default, item.name.casefold()))
            return loopbacks, microphones
        finally:
            audio.terminate()

    def _default_index(self, audio, input_device: bool) -> int | None:
        try:
            info = audio.get_default_input_device_info() if input_device else audio.get_default_output_device_info()
            return int(info["index"])
        except Exception:
            return None

    def _default_loopback_index(self, audio, loopbacks: list[AudioDevice]) -> int | None:
        if not loopbacks:
            return None
        try:
            output = audio.get_default_output_device_info()
            output_name = str(output.get("name", "")).casefold()
        except Exception:
            return loopbacks[0].index
        for device in loopbacks:
            loop_name = device.name.casefold()
            if output_name and (output_name in loop_name or loop_name in output_name):
                return device.index
        return loopbacks[0].index

    def resolve(self, kind: str, identifier: str | None = None) -> AudioDevice | None:
        systems, microphones = self.enumerate()
        candidates = systems if kind == "system" else microphones
        if identifier:
            for device in candidates:
                if device.identifier == identifier:
                    return device
        return candidates[0] if candidates else None
