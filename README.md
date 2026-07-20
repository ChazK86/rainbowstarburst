# Rainbow Starburst

This repository contains a pyglet + PyOpenGL visualization driven by `mastercontroller.py` and a tactile switchboard-style UI in `pyramidGUI.py`.

## Prerequisites

The project targets Python 3.10+ and depends on:

- [numpy](https://numpy.org/)
- [pyglet](https://pyglet.org/)
- [PyOpenGL](http://pyopengl.sourceforge.net/)
- [PyAudioWPatch](https://github.com/s0d3s/PyAudioWPatch) on Windows for WASAPI loopback and microphone capture

Install them into your active environment (virtualenv or system Python):

```bash
pip install -r requirements.txt
```

`requirements.txt` installs the Windows audio backend conditionally, so the
same installation command remains safe on other operating systems.

On Linux you may also need system OpenGL drivers and an X11 session (or Wayland with XWayland) so pyglet can create windows.

## Running the application

The interactive entry point is the GUI module:

```bash
python pyramidGUI.py
```

This launches two pyglet windows and immediately routes the animated **Star** formation so the stage is never empty:

1. **Switchboard console** &mdash; formation routing, local audio sources, seven audio programs, manual motion, particle limits, response controls, endpoint selection, and live diagnostics.
2. **3D visualization** &mdash; renders immutable home geometry plus bounded per-pyramid audio modulation, a moving rainbow signal path, a depth grid, and particles.

Arrange both windows so they stay visible; closing either one exits the process.

## Controls

- **Arrangements**: Choose Edge2Edge, SpikeSphere, Grid, Star, or Globe to rebuild the scene. SpikeSphere spaces square pyramids over a sphere. Star is a full joined SpikeSphere: its triangular bases share edges across an icosahedral core and all apexes point outward. Globe detail is safely bounded to subdivisions 0&ndash;3 (20&ndash;1,280 faces), and the turntable sets apex offset.
- **Audio sources**: OFF, SYSTEM, MIC, BOTH, and DEMO. Audio always starts OFF. SYSTEM follows a selected Windows WASAPI loopback endpoint, MIC opens the selected input, BOTH analyzes two independent streams, and DEMO generates a deterministic private test signal.
- **Audio programs**: CROWN, BLOOM, ORBIT, CINEMA, RADAR, RELAY, and AURORA. Each program controls individual pyramid tips, routing, line response, and particles through the same bounded render-state contract.
- **Manual animations**: WAVE/SPIN/PULSE/NONE remain available and mutually exclusive. Selecting manual motion stops audio capture and immediately returns reactive geometry home.
- **Particle modes**: OFF/LOW/MEDIUM/HEAVY control click-burst intensity. Program particles remain globally capped at 2,000.
- **Response controls**: Sensitivity controls audio gain (and manual wave depth), Reactivity controls envelope speed, Globe Detail sets the next globe density, and Apex Offset changes the next globe's tip height.
- **Endpoint controls**: NEXT SYSTEM and NEXT MIC cycle the discovered endpoints. Selected identifiers and response preferences are stored in the current user's local application-data folder; audio never auto-starts on launch.
- **Reduced motion**: Scales transient displacement, rotation, and particle output while preserving analysis and program routing.
- **Viewport**: Click the visualization to emit the selected particle burst, scroll to move the camera, press Space to pause, or press R to reset the trace and camera.
- **Keyboard**: Keys 1&ndash;5 route the five formations; Escape closes both windows.

Closing either window exits the complete application.

## Data output

Each arrangement writes its definitions to `Pyramids/`. After every switch, files matching `pyramid_<integer>.txt` mirror the active formation exactly: stale higher IDs are removed and the current IDs are rewritten. Other files and subdirectories are preserved. Exporting can still be disabled or redirected programmatically through `MasterController`.

The switchboard defaults produce these managed file counts:

| Formation | Files |
| --- | ---: |
| Edge2Edge | 7 |
| SpikeSphere | 24 |
| Grid | 20 |
| Star | 80 |
| Globe | `20 × 4^detail` (20, 80, 320, or 1,280) |

## Audio architecture and privacy

- Raw samples stay in bounded memory buffers and are discarded after analysis. The application does not record, transcribe, upload, or transmit audio.
- PortAudio callbacks only copy samples. Analyzer threads publish immutable feature frames; only the pyglet render thread changes visual state.
- System output and microphone use separate sample rates, noise floors, normalization, and analyzers. They are combined at the feature level instead of mixing unsynchronized PCM.
- Endpoint loss degrades to an unavailable status while the renderer continues. A monitor retries the selected endpoint without blocking a frame.
- Every program returns exactly to authored home geometry after 1.5 seconds of silence. Reactive state is never written into pyramid exports.

The detailed algorithms, mappings, controls, and acceptance criteria are in
[docs/AUDIO_REACTIVE_ROADMAP.md](docs/AUDIO_REACTIVE_ROADMAP.md).

## Tests

Run the complete geometry, audio-feature, animation, lifecycle, and export suite with:

```bash
python -m unittest discover -s tests -v
```

The deterministic suite uses generated signals only; it never captures test audio
from the local computer.

List endpoints or inspect live normalized levels without opening the GUI:

```bash
python -m audio.diagnostic --list
python -m audio.diagnostic --source both --seconds 10
python -m audio.diagnostic --source demo --seconds 5
```
