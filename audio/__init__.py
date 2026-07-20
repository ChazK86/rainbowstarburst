"""Local, in-memory audio analysis for Rainbow Starburst."""

from .engine import AudioEngine, AudioSourceMode
from .features import AudioFeatureFrame, FeatureAnalyzer, SourceFeatures

__all__ = [
    "AudioEngine",
    "AudioSourceMode",
    "AudioFeatureFrame",
    "FeatureAnalyzer",
    "SourceFeatures",
]
