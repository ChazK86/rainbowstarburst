# Rainbow Starburst

This repository contains a pyglet + PyOpenGL visualization driven by `mastercontroller.py` and a tactile switchboard-style UI in `pyramidGUI.py`.

## Prerequisites

The project targets Python 3.9+ and depends on:

- [numpy](https://numpy.org/)
- [pyglet](https://pyglet.org/)
- [PyOpenGL](http://pyopengl.sourceforge.net/)

Install them into your active environment (virtualenv or system Python):

```bash
pip install -r requirements.txt
```

or directly:

```bash
pip install numpy pyglet PyOpenGL
```

On Linux you may also need system OpenGL drivers and an X11 session (or Wayland with XWayland) so pyglet can create windows.

## Running the application

The interactive entry point is the GUI module:

```bash
python pyramidGUI.py
```

This launches two pyglet windows and immediately routes the animated **Star** formation so the stage is never empty:

1. **Switchboard console** &mdash; illuminated controls for arrangements, particle gain, and animation programs plus bounded knobs for wave strength and globe detail.
2. **3D visualization** &mdash; renders the active formation, its animated rainbow signal path, a depth grid, and interactive particle bursts.

Arrange both windows so they stay visible; closing either one exits the process.

## Controls

- **Arrangements**: Choose Edge2Edge, SpikeSphere, Grid, Star, or Globe to rebuild the scene. SpikeSphere spaces square pyramids over a sphere. Star is a full joined SpikeSphere: its triangular bases share edges across an icosahedral core and all apexes point outward. Globe detail is safely bounded to subdivisions 0&ndash;3 (20&ndash;1,280 faces), and the turntable sets apex offset.
- **Particle Modes**: OFF/LOW/MEDIUM/HEAVY toggle burst intensity. Modes are handled by `MasterController.set_particle_mode`.
- **Animations**: WAVE/SPIN/PULSE/NONE are mutually exclusive motion programs. The signal monitor changes color with the active program.
- **Knobs**: Drag Wave Amplitude to scale non-accumulating wave motion; drag Globe Detail to set the next globe density. Spin Apex Offset before rebuilding the globe.
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

## Tests

Run the geometry, animation, and export synchronization checks with:

```bash
python -m unittest discover -s tests -v
```

## Next: local audio reactivity

The implementation-ready plan for Windows system-output capture, microphone input, per-pyramid modulation architecture, and seven creative reactive programs is in [docs/AUDIO_REACTIVE_ROADMAP.md](docs/AUDIO_REACTIVE_ROADMAP.md).
