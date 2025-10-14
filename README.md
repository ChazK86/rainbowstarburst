# Rainbow Starburst

This repository contains a pyglet + PyOpenGL visualization driven by `mastercontroller.py` and a switchboard-style UI in `pyramidGUI.py`.

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

This launches two pyglet windows:

1. **Switchboard console** &mdash; buttons for changing arrangements, particle density, and animation modes along with knobs to tweak wave strength and globe subdivisions.
2. **3D visualization** &mdash; renders the active pyramid arrangement and a rainbow path while continuously calling `MasterController.update` for physics and export scheduling.

Arrange both windows so they stay visible; closing either one exits the process.

## Controls

- **Arrangements**: Choose Edge2Edge, SpikeSphere, Grid, Star, or Globe to rebuild the scene. Globe uses the subdivision knob and turntable to adjust density and apex offset respectively.
- **Particle Modes**: OFF/LOW/MEDIUM/HEAVY toggle burst intensity. Modes are handled by `MasterController.set_particle_mode`.
- **Animations**: WAVE/SPIN/PULSE/NONE set the animation mode. The LED indicator above the speaker rectangle lights up in green/red/blue depending on the active mode.
- **Knobs**: Drag WaveAmp to scale wave motion on the Y axis for pyramids that enable it; drag GlobeSubdiv to change the next globe arrangement density. Spin the turntable disc to update the apex offset before rebuilding the globe.

Mouse interactions are bound inside the UI window; the visualization window responds to expose events and automatically spins the camera. Use the console output for feedback on knob/arrangement changes.

## Data output

Each time the controller creates pyramids it writes their definitions to `Pyramids/` using the deferred export queue configured in `MasterController`. You can disable exporting or change throughput programmatically before starting intensive sessions.

