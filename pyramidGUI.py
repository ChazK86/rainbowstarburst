#!/usr/bin/env python3
"""Rainbow Starburst switchboard and live audio-reactive OpenGL stage."""

from __future__ import annotations

import colorsys
import math
import random
import time

import numpy as np
import pyglet
from pyglet import shapes
from pyglet.window import key, mouse
from OpenGL.GL import (
    GL_BLEND,
    GL_COLOR_BUFFER_BIT,
    GL_CULL_FACE,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_LINES,
    GL_LINE_STRIP,
    GL_MODELVIEW,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_POINTS,
    GL_PROJECTION,
    GL_SRC_ALPHA,
    glBegin,
    glBlendFunc,
    glClear,
    glClearColor,
    glColor4f,
    glDisable,
    glEnable,
    glEnd,
    glLineWidth,
    glLoadIdentity,
    glMatrixMode,
    glPointSize,
    glRotatef,
    glVertex3f,
    glViewport,
)
from OpenGL.GLU import gluLookAt, gluPerspective

from animations import AnimationDirector, PROGRAM_NAMES, apply_render_state
from audio import AudioEngine, AudioSourceMode
from mastercontroller import (
    ANIMATION_PULSE,
    ANIMATION_SPIN,
    ANIMATION_WAVE_Y,
    MAX_GLOBE_SUBDIVISIONS,
    PARTICLE_HEAVY,
    PARTICLE_LOW,
    PARTICLE_MEDIUM,
    PARTICLE_OFF,
    MasterController,
)
from settings import load_settings, save_settings


PALETTE = {
    "ink": (11, 16, 21),
    "panel": (25, 31, 36),
    "panel_raised": (38, 45, 51),
    "panel_hover": (51, 61, 68),
    "line": (75, 85, 91),
    "cream": (240, 229, 199),
    "muted": (157, 163, 155),
    "brass": (202, 164, 91),
    "amber": (255, 180, 73),
    "teal": (68, 202, 190),
    "red": (235, 92, 86),
    "blue": (94, 142, 236),
}

ARRANGEMENTS = ("Edge2Edge", "SpikeSphere", "Grid", "Star", "Globe")
SOURCE_LABELS = ("OFF", "SYSTEM", "MIC", "BOTH", "DEMO")
PARTICLE_MODES = (PARTICLE_OFF, PARTICLE_LOW, PARTICLE_MEDIUM, PARTICLE_HEAVY)
ANIMATION_LABELS = ("WAVE", "SPIN", "PULSE", "NONE")
ANIMATION_VALUES = {
    "WAVE": ANIMATION_WAVE_Y,
    "SPIN": ANIMATION_SPIN,
    "PULSE": ANIMATION_PULSE,
    "NONE": "",
}

mc = MasterController()
audio_engine = AudioEngine()
director = AnimationDirector()
preferences = load_settings()

ui_window = None
visualization_window = None
knob_wave_value = 0.8
knob_detail_value = 1.0
turntable_angle = 0.0


def _rgba(rgb, alpha=255):
    return (*rgb, alpha)


def _rainbow_color(fraction, saturation=0.86, value=1.0):
    return colorsys.hsv_to_rgb(float(fraction) % 1.0, saturation, value)


def _short_name(value, limit=30):
    text = str(value)
    return text if len(text) <= limit else text[: limit - 1] + "…"


class ToggleButton:
    """A compact, keyboard-like switch with one clear active state."""

    def __init__(self, x, y, width, height, text, on_press, batch, accent=None):
        self.x, self.y, self.width, self.height = x, y, width, height
        self.on_press = on_press
        self.accent = accent or PALETTE["amber"]
        self.active = False
        self.hovered = False
        self.border = shapes.Rectangle(x - 1, y - 1, width + 2, height + 2, color=PALETTE["line"], batch=batch)
        self.rect = shapes.Rectangle(x, y, width, height, color=PALETTE["panel_raised"], batch=batch)
        self.signal = shapes.Rectangle(x, y, 3, height, color=PALETTE["line"], batch=batch)
        self.txt = pyglet.text.Label(
            text,
            x=x + width / 2,
            y=y + height / 2,
            anchor_x="center",
            anchor_y="center",
            font_name="Segoe UI",
            font_size=8,
            color=_rgba(PALETTE["cream"]),
            batch=batch,
        )

    def hit_test(self, x, y):
        return self.x <= x <= self.x + self.width and self.y <= y <= self.y + self.height

    def set_active(self, active):
        self.active = bool(active)
        self._refresh()

    def set_text(self, value):
        self.txt.text = str(value)

    def on_mouse_motion(self, x, y):
        hovered = self.hit_test(x, y)
        if hovered != self.hovered:
            self.hovered = hovered
            self._refresh()

    def on_mouse_press(self, x, y, button, modifiers):
        del modifiers
        if button == mouse.LEFT and self.hit_test(x, y):
            self.on_press()
            return True
        return False

    def _refresh(self):
        if self.active:
            self.border.color = self.accent
            self.rect.color = self.accent
            self.signal.color = PALETTE["cream"]
            self.txt.color = _rgba(PALETTE["ink"])
        else:
            self.border.color = PALETTE["brass"] if self.hovered else PALETTE["line"]
            self.rect.color = PALETTE["panel_hover"] if self.hovered else PALETTE["panel_raised"]
            self.signal.color = self.accent if self.hovered else PALETTE["line"]
            self.txt.color = _rgba(PALETTE["cream"])


class Knob:
    """Bounded rotary parameter with a live numeric readout."""

    def __init__(self, x, y, radius, label, batch, on_drag, minimum, maximum, value, step, value_format):
        self.x, self.y, self.r = x, y, radius
        self.minimum, self.maximum = float(minimum), float(maximum)
        self.on_drag, self.step, self.value_format = on_drag, step, value_format
        self.value = float(value)
        self.dragging = False
        self.halo = shapes.Circle(x, y, radius + 4, color=PALETTE["line"], batch=batch)
        self.bg = shapes.Circle(x, y, radius, color=PALETTE["panel_raised"], batch=batch)
        self.cap = shapes.Circle(x, y, max(7, radius - 12), color=PALETTE["panel"], batch=batch)
        self.indicator = shapes.Line(x, y, x, y + radius - 7, thickness=3, color=PALETTE["amber"], batch=batch)
        self.label = pyglet.text.Label(
            label.upper(), x=x, y=y + radius + 18, anchor_x="center", anchor_y="center",
            font_name="Segoe UI", font_size=8, color=_rgba(PALETTE["muted"]), batch=batch,
        )
        self.value_label = pyglet.text.Label(
            "", x=x, y=y - radius - 16, anchor_x="center", anchor_y="center",
            font_name="Consolas", font_size=9, color=_rgba(PALETTE["cream"]), batch=batch,
        )
        self.set_value(value, notify=False)

    def hit_test(self, x, y):
        return math.hypot(x - self.x, y - self.y) <= self.r + 5

    def on_mouse_press(self, x, y, button, modifiers):
        del modifiers
        if button == mouse.LEFT and self.hit_test(x, y):
            self.dragging = True
            self._set_from_pointer(x, y)
            return True
        return False

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        del dx, dy, modifiers
        if self.dragging and buttons & mouse.LEFT:
            self._set_from_pointer(x, y)

    def on_mouse_release(self, x, y, button, modifiers):
        del x, y, modifiers
        if button == mouse.LEFT:
            self.dragging = False

    def set_value(self, value, notify=True):
        value = max(self.minimum, min(self.maximum, float(value)))
        if self.step:
            value = round(value / self.step) * self.step
        self.value = value
        fraction = (value - self.minimum) / max(1e-9, self.maximum - self.minimum)
        angle = math.radians(225.0 + fraction * 270.0)
        length = self.r - 7
        self.indicator.x2 = self.x + math.cos(angle) * length
        self.indicator.y2 = self.y + math.sin(angle) * length
        self.value_label.text = self.value_format.format(value)
        if notify:
            self.on_drag(value)

    def _set_from_pointer(self, x, y):
        angle = math.degrees(math.atan2(y - self.y, x - self.x)) % 360.0
        relative = (angle - 225.0) % 360.0
        if relative > 270.0:
            relative = 0.0 if relative > 315.0 else 270.0
        self.set_value(self.minimum + relative / 270.0 * (self.maximum - self.minimum))


class Turntable:
    def __init__(self, x, y, radius, label, batch, on_spin):
        self.x, self.y, self.r = x, y, radius
        self.on_spin = on_spin
        self.dragging = False
        self.angle = 0.0
        self.outer = shapes.Circle(x, y, radius + 5, color=PALETTE["brass"], batch=batch)
        self.bg = shapes.Circle(x, y, radius, color=(28, 37, 47), batch=batch)
        self.groove = shapes.Circle(x, y, radius - 10, color=(19, 26, 32), batch=batch)
        self.hub = shapes.Circle(x, y, 7, color=PALETTE["cream"], batch=batch)
        self.indicator = shapes.Line(x, y, x + radius - 10, y, thickness=3, color=PALETTE["amber"], batch=batch)
        self.label = pyglet.text.Label(
            label.upper(), x=x, y=y + radius + 18, anchor_x="center", anchor_y="center",
            font_name="Segoe UI", font_size=8, color=_rgba(PALETTE["muted"]), batch=batch,
        )
        self.value_label = pyglet.text.Label(
            "000°", x=x, y=y - radius - 16, anchor_x="center", anchor_y="center",
            font_name="Consolas", font_size=9, color=_rgba(PALETTE["cream"]), batch=batch,
        )

    def hit_test(self, x, y):
        return math.hypot(x - self.x, y - self.y) <= self.r + 5

    def on_mouse_press(self, x, y, button, modifiers):
        del modifiers
        if button == mouse.LEFT and self.hit_test(x, y):
            self.dragging = True
            self._set(x, y)
            return True
        return False

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        del dx, dy, modifiers
        if self.dragging and buttons & mouse.LEFT:
            self._set(x, y)

    def on_mouse_release(self, x, y, button, modifiers):
        del x, y, modifiers
        if button == mouse.LEFT:
            self.dragging = False

    def _set(self, x, y):
        self.angle = math.degrees(math.atan2(y - self.y, x - self.x)) % 360.0
        radians = math.radians(self.angle)
        length = self.r - 10
        self.indicator.x2 = self.x + math.cos(radians) * length
        self.indicator.y2 = self.y + math.sin(radians) * length
        self.value_label.text = f"{int(round(self.angle)) % 360:03d}°"
        self.on_spin(self.angle)


class SwitchboardWindow(pyglet.window.Window):
    """A restrained studio console around one dominant live visual canvas."""

    def __init__(self, width=840, height=760, title="Rainbow Starburst — Switchboard"):
        super().__init__(width, height, title, resizable=False, vsync=True)
        self.batch = pyglet.graphics.Batch()
        self.labels = []
        self.controls = []
        self.formation_buttons = []
        self.source_buttons = []
        self.program_buttons = []
        self.motion_buttons = []
        self.particle_buttons = []
        self.active_arrangement = "Star"
        self.active_source = "OFF"  # Never auto-open a saved microphone.
        self.active_program = preferences.audio_program if preferences.audio_program in PROGRAM_NAMES else "CINEMA"
        self.active_animation = "WAVE"
        self.active_particle = PARTICLE_OFF
        self.system_devices, self.microphone_devices = audio_engine.devices()

        shapes.Rectangle(0, 0, width, height, color=PALETTE["ink"], batch=self.batch)
        shapes.Rectangle(20, 18, width - 40, height - 36, color=PALETTE["panel"], batch=self.batch)
        shapes.Rectangle(40, 668, width - 80, 2, color=PALETTE["brass"], batch=self.batch)
        self._label("RAINBOW / STARBURST", 40, 716, 22, PALETTE["cream"], anchor_y="center")
        self._label("PYRAMID SIGNAL CONSOLE  •  RS–02", 42, 690, 9, PALETTE["muted"], anchor_y="center")
        self.live_dot = shapes.Circle(width - 142, 704, 6, color=PALETTE["line"], batch=self.batch)
        self.live_label = self._label("AUDIO OFF", width - 126, 704, 9, PALETTE["muted"], anchor_y="center")

        self._section("01  FORMATION ROUTING", 40, 644)
        self._button_row(ARRANGEMENTS, 40, 590, 144, 8, self.formation_buttons, self.on_arrangement, PALETTE["amber"])
        self._section("02  LOCAL AUDIO SOURCE", 40, 560)
        self._button_row(SOURCE_LABELS, 40, 506, 144, 8, self.source_buttons, self.on_source, PALETTE["teal"])
        self._section("03  AUDIO-REACTIVE PROGRAM", 40, 476)
        self._button_row(PROGRAM_NAMES, 40, 422, 100, 8, self.program_buttons, self.on_program, PALETTE["blue"])

        self._section("04  MANUAL MOTION", 40, 391)
        self._button_row(ANIMATION_LABELS, 40, 338, 80, 8, self.motion_buttons, self.on_manual_motion, PALETTE["amber"])
        self._section("05  PARTICLE LIMIT", 446, 391)
        self._button_row(PARTICLE_MODES, 446, 338, 74, 8, self.particle_buttons, self.on_particle, PALETTE["teal"])

        shapes.Rectangle(40, 306, width - 80, 1, color=PALETTE["line"], batch=self.batch)
        self._section("06  RESPONSE + DEVICE STATUS", 40, 284)
        self.sensitivity_knob = Knob(
            94, 191, 36, "Sensitivity", self.batch, self.on_sensitivity,
            0.5, 2.0, preferences.sensitivity, 0.05, "{:.2f}",
        )
        self.reactivity_knob = Knob(
            214, 191, 36, "Reactivity", self.batch, self.on_reactivity,
            0.55, 1.6, preferences.reactivity, 0.05, "{:.2f}",
        )
        self.detail_knob = Knob(
            334, 191, 36, "Globe detail", self.batch, self.on_detail,
            0, MAX_GLOBE_SUBDIVISIONS, knob_detail_value, 1, "{:.0f}",
        )
        self.apex_turntable = Turntable(460, 188, 45, "Apex offset", self.batch, self.on_apex)
        self.controls.extend((self.sensitivity_knob, self.reactivity_knob, self.detail_knob, self.apex_turntable))

        self.system_name = self._label("SYSTEM  —", 548, 250, 8, PALETTE["muted"])
        self.system_meter_bg = shapes.Rectangle(548, 230, 244, 5, color=(48, 56, 61), batch=self.batch)
        self.system_meter = shapes.Rectangle(548, 230, 0, 5, color=PALETTE["teal"], batch=self.batch)
        self.mic_name = self._label("MIC     —", 548, 208, 8, PALETTE["muted"])
        self.mic_meter_bg = shapes.Rectangle(548, 188, 244, 5, color=(48, 56, 61), batch=self.batch)
        self.mic_meter = shapes.Rectangle(548, 188, 0, 5, color=PALETTE["amber"], batch=self.batch)
        self.telemetry = self._label("0.0 MS  •  0 DROPPED", 548, 164, 8, PALETTE["muted"])
        self.privacy = self._label("LIVE ANALYSIS — NOT RECORDING", 548, 143, 8, PALETTE["brass"])
        self.system_device_button = ToggleButton(548, 94, 118, 32, "NEXT SYSTEM", self.cycle_system_device, self.batch, PALETTE["teal"])
        self.mic_device_button = ToggleButton(674, 94, 118, 32, "NEXT MIC", self.cycle_microphone_device, self.batch, PALETTE["amber"])
        self.reduced_button = ToggleButton(548, 54, 244, 30, "REDUCED MOTION", self.toggle_reduced_motion, self.batch, PALETTE["blue"])
        self.controls.extend((self.system_device_button, self.mic_device_button, self.reduced_button))

        self.status_label = self._label("", 40, 34, 8, PALETTE["muted"], anchor_y="center")
        self.help_label = self._label(
            "1–5 FORMATIONS  •  SPACE PAUSES VIEW", width - 40, 34, 8, PALETTE["muted"],
            anchor_x="right", anchor_y="center",
        )
        self._sync_buttons()
        self._update_device_names()
        self.reduced_button.set_active(bool(preferences.reduced_motion))
        director.sensitivity = float(preferences.sensitivity)
        director.reactivity = float(preferences.reactivity)
        director.reduced_motion = bool(preferences.reduced_motion)
        self.set_status("STAR ROUTED  •  MANUAL WAVE ACTIVE")
        pyglet.clock.schedule_interval(self.update_audio_status, 0.1)

    def _label(self, text, x, y, size, color, anchor_x="left", anchor_y="baseline"):
        label = pyglet.text.Label(
            text, x=x, y=y, anchor_x=anchor_x, anchor_y=anchor_y,
            font_name="Segoe UI", font_size=size, color=_rgba(color), batch=self.batch,
        )
        self.labels.append(label)
        return label

    def _section(self, text, x, y):
        return self._label(text, x, y, 9, PALETTE["brass"])

    def _button_row(self, values, x, y, width, gap, collection, callback, accent):
        for index, value in enumerate(values):
            button = ToggleButton(
                x + index * (width + gap), y, width, 38, str(value),
                lambda selected=value: callback(selected), self.batch, accent,
            )
            collection.append(button)
            self.controls.append(button)

    def _sync_buttons(self):
        for button, value in zip(self.formation_buttons, ARRANGEMENTS):
            button.set_active(value == self.active_arrangement)
        for button, value in zip(self.source_buttons, SOURCE_LABELS):
            button.set_active(value == self.active_source)
        for button, value in zip(self.program_buttons, PROGRAM_NAMES):
            button.set_active(value == self.active_program)
        for button, value in zip(self.motion_buttons, ANIMATION_LABELS):
            button.set_active(value == self.active_animation)
        for button, value in zip(self.particle_buttons, PARTICLE_MODES):
            button.set_active(value == self.active_particle)

    def _save(self):
        preferences.audio_source = self.active_source
        preferences.audio_program = self.active_program
        preferences.system_device_id = audio_engine.system_device_id
        preferences.microphone_device_id = audio_engine.microphone_device_id
        preferences.sensitivity = director.sensitivity
        preferences.reactivity = director.reactivity
        preferences.reduced_motion = director.reduced_motion
        try:
            save_settings(preferences)
        except OSError:
            pass

    def set_status(self, message):
        self.status_label.text = str(message)

    def on_draw(self):
        glClearColor(*(channel / 255.0 for channel in PALETTE["ink"]), 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.batch.draw()

    def on_mouse_motion(self, x, y, dx, dy):
        del dx, dy
        for control in self.controls:
            if isinstance(control, ToggleButton):
                control.on_mouse_motion(x, y)

    def on_mouse_press(self, x, y, button, modifiers):
        for control in self.controls:
            if control.on_mouse_press(x, y, button, modifiers):
                return

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        for control in self.controls:
            if isinstance(control, (Knob, Turntable)):
                control.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

    def on_mouse_release(self, x, y, button, modifiers):
        for control in self.controls:
            if isinstance(control, (Knob, Turntable)):
                control.on_mouse_release(x, y, button, modifiers)

    def on_key_press(self, symbol, modifiers):
        del modifiers
        formations = {key._1: "Edge2Edge", key._2: "SpikeSphere", key._3: "Grid", key._4: "Star", key._5: "Globe"}
        if symbol in formations:
            self.on_arrangement(formations[symbol])
        elif symbol == key.ESCAPE:
            self._shutdown()

    def on_arrangement(self, label):
        if label == "Edge2Edge":
            mc.init_edge_to_edge_pyramids(count=7)
        elif label == "SpikeSphere":
            mc.init_spike_sphere_pyramids(count=24, sphere_radius=4.0)
        elif label == "Grid":
            mc.init_grid_pyramids(rows=4, cols=5, spacing_x=1.8, spacing_z=1.8)
        elif label == "Star":
            mc.init_star_formation(subdivisions=1, core_radius=3.0, spike_height=2.4)
        elif label == "Globe":
            detail = int(round(self.detail_knob.value))
            offset = 0.18 + (turntable_angle % 360.0) / 360.0 * 0.7
            mc.init_globe_icosahedron(subdivisions=detail, apex_offset=offset, base_scale=4.0)
        self.active_arrangement = label
        if self.active_source == "OFF":
            mc.set_animation_mode(ANIMATION_VALUES[self.active_animation])
            self._apply_wave()
        else:
            mc.set_animation_mode("")
        director.set_formation(mc.pyramids)
        self._sync_buttons()
        self.set_status(f"{label.upper()} ROUTED  •  {len(mc.pyramids)} PYRAMIDS")
        if visualization_window:
            visualization_window.set_scene(label)

    def on_source(self, label):
        self.active_source = label
        if label == "OFF":
            audio_engine.configure(AudioSourceMode.OFF)
            director.select(None)
            self.live_dot.color = PALETTE["line"]
            self.live_label.text = "AUDIO OFF"
            self.live_label.color = _rgba(PALETTE["muted"])
            self.set_status("AUDIO ANALYSIS OFF  •  HOME GEOMETRY RESTORED")
        else:
            mc.set_animation_mode("")
            self.active_animation = "NONE"
            audio_engine.configure(
                label,
                system_device_id=preferences.system_device_id,
                microphone_device_id=preferences.microphone_device_id,
            )
            director.select(self.active_program)
            self.live_dot.color = PALETTE["teal"]
            self.live_label.text = f"{self.active_program} LIVE"
            self.live_label.color = _rgba(PALETTE["teal"])
            self.set_status(f"{label} ANALYSIS STARTED  •  NOT RECORDING")
        self._sync_buttons()
        self._save()

    def on_program(self, label):
        self.active_program = label
        if self.active_source != "OFF":
            director.select(label)
            self.live_label.text = f"{label} LIVE"
            self.set_status(f"AUDIO PROGRAM {label}  •  250 MS COHESIVE TRANSITION")
        else:
            self.set_status(f"{label} ARMED  •  SELECT AN AUDIO SOURCE")
        self._sync_buttons()
        self._save()

    def on_manual_motion(self, label):
        self.active_source = "OFF"
        audio_engine.configure(AudioSourceMode.OFF)
        director.select(None)
        self.active_animation = label
        mc.set_animation_mode(ANIMATION_VALUES[label])
        self._apply_wave()
        self.live_dot.color = PALETTE["line"]
        self.live_label.text = "AUDIO OFF"
        self.live_label.color = _rgba(PALETTE["muted"])
        self._sync_buttons()
        self._save()
        self.set_status(f"MANUAL MOTION {label}")

    def on_particle(self, mode):
        self.active_particle = mode
        mc.set_particle_mode(mode)
        self._sync_buttons()
        self.set_status(f"PARTICLE LIMIT {mode}")

    def on_sensitivity(self, value):
        global knob_wave_value
        director.sensitivity = float(value)
        knob_wave_value = float(value)
        self._apply_wave()
        self._save()
        self.set_status(f"SENSITIVITY {value:.2f}")

    def on_reactivity(self, value):
        director.reactivity = float(value)
        self._save()
        self.set_status(f"REACTIVITY {value:.2f}")

    def on_detail(self, value):
        global knob_detail_value
        knob_detail_value = float(round(value))
        faces = 20 * (4 ** int(knob_detail_value))
        self.set_status(f"GLOBE DETAIL {int(knob_detail_value)}  •  {faces} FACES ON NEXT ROUTE")

    def on_apex(self, angle):
        global turntable_angle
        turntable_angle = float(angle)
        offset = 0.18 + (turntable_angle % 360.0) / 360.0 * 0.7
        self.set_status(f"GLOBE APEX OFFSET {offset:.2f}")

    def _apply_wave(self):
        if self.active_animation not in ("WAVE", "PULSE"):
            return
        multiplier = 1.65 if self.active_animation == "PULSE" else 1.0
        for pyramid in mc.pyramids:
            if pyramid.physics.wave_axis_enable["y"]:
                pyramid.physics.wave_amplitude["y"] = knob_wave_value * multiplier

    def cycle_system_device(self):
        self.system_devices, self.microphone_devices = audio_engine.devices()
        preferences.system_device_id = self._next_device(self.system_devices, preferences.system_device_id)
        self._update_device_names()
        if self.active_source in ("SYSTEM", "BOTH"):
            self.on_source(self.active_source)
        self._save()

    def cycle_microphone_device(self):
        self.system_devices, self.microphone_devices = audio_engine.devices()
        preferences.microphone_device_id = self._next_device(self.microphone_devices, preferences.microphone_device_id)
        self._update_device_names()
        if self.active_source in ("MIC", "BOTH"):
            self.on_source(self.active_source)
        self._save()

    @staticmethod
    def _next_device(devices, current):
        if not devices:
            return None
        ids = [item.identifier for item in devices]
        return ids[(ids.index(current) + 1) % len(ids)] if current in ids else ids[0]

    def _update_device_names(self):
        system = next((item for item in self.system_devices if item.identifier == preferences.system_device_id), None)
        microphone = next((item for item in self.microphone_devices if item.identifier == preferences.microphone_device_id), None)
        if system is None and self.system_devices:
            system = self.system_devices[0]
            preferences.system_device_id = system.identifier
        if microphone is None and self.microphone_devices:
            microphone = self.microphone_devices[0]
            preferences.microphone_device_id = microphone.identifier
        self.system_name.text = "SYSTEM  " + _short_name(system.name if system else "NO LOOPBACK ENDPOINT")
        self.mic_name.text = "MIC     " + _short_name(microphone.name if microphone else "NO INPUT ENDPOINT")

    def toggle_reduced_motion(self):
        director.reduced_motion = not director.reduced_motion
        self.reduced_button.set_active(director.reduced_motion)
        self._save()
        self.set_status("REDUCED MOTION " + ("ON" if director.reduced_motion else "OFF"))

    def update_audio_status(self, dt):
        del dt
        frame = audio_engine.snapshot()
        status = audio_engine.status()
        self.system_meter.width = int(244 * frame.system.rms)
        self.mic_meter.width = int(244 * frame.microphone.rms)
        self.system_meter.color = PALETTE["teal"] if frame.system.active else PALETTE["line"]
        self.mic_meter.color = PALETTE["amber"] if frame.microphone.active else PALETTE["line"]
        self.telemetry.text = (
            f"{frame.capture_latency_ms:4.1f} MS  •  {frame.dropped_windows} DROPPED  "
            f"•  {len(mc.pyramids)} RESONANT VOICES"
        )
        if self.active_source != "OFF":
            system_state = status["system"].state
            mic_state = status["microphone"].state
            if self.active_source in ("SYSTEM", "BOTH") and system_state == "unavailable":
                self.live_dot.color = PALETTE["red"]
                self.live_label.text = "SYSTEM UNAVAILABLE"
            elif self.active_source in ("MIC", "BOTH") and mic_state == "unavailable":
                self.live_dot.color = PALETTE["red"]
                self.live_label.text = "MIC UNAVAILABLE"

    def _shutdown(self):
        pyglet.clock.unschedule(self.update_audio_status)
        audio_engine.shutdown()
        pyglet.app.exit()

    def on_close(self):
        self._shutdown()


class VisualizationWindow(pyglet.window.Window):
    """The dominant live canvas; wireframes rest white and react individually."""

    def __init__(self, width=900, height=650, title="Rainbow Starburst — 3D Signal"):
        super().__init__(width, height, title, resizable=True, vsync=True)
        self.set_minimum_size(560, 420)
        self.start_time = time.time()
        self.draw_distance = 0.0
        self.draw_speed = 10.0
        self.camera_distance = 14.0
        self.scene_name = "Star"
        self.paused = False
        self.cursor_world = np.zeros(3)
        self.cursor_visible = False
        self.particle_accumulator = np.zeros(len(mc.pyramids))
        rng = random.Random(86)
        self.stars = [
            (rng.uniform(-14, 14), rng.uniform(-6, 11), rng.uniform(-12, -2), rng.uniform(0.25, 0.8))
            for _ in range(120)
        ]
        self.hud = pyglet.graphics.Batch()
        self.hud_brand = pyglet.text.Label(
            "RAINBOW / STARBURST", x=26, y=height - 30, anchor_y="top",
            font_name="Segoe UI", font_size=13, color=_rgba(PALETTE["cream"]), batch=self.hud,
        )
        self.hud_scene = pyglet.text.Label(
            "", x=26, y=height - 55, anchor_y="top", font_name="Consolas",
            font_size=9, color=_rgba(PALETTE["teal"]), batch=self.hud,
        )
        self.hud_help = pyglet.text.Label(
            "CLICK: PARTICLE BURST   •   SCROLL: CAMERA   •   SPACE: PAUSE",
            x=26, y=22, anchor_y="bottom", font_name="Segoe UI", font_size=8,
            color=_rgba(PALETTE["muted"]), batch=self.hud,
        )
        self._update_hud()
        pyglet.clock.schedule_interval(self.update, 1.0 / 60.0)

    def set_scene(self, name):
        self.scene_name = name
        self.draw_distance = 0.0
        self.particle_accumulator = np.zeros(len(mc.pyramids))
        self._update_hud()

    def _update_hud(self):
        mode = director.active_name or (ui_window.active_animation if ui_window else "MANUAL")
        source = ui_window.active_source if ui_window else "OFF"
        paused = "  •  PAUSED" if self.paused else ""
        self.hud_scene.text = f"{self.scene_name.upper()}  •  {len(mc.pyramids)} PYRAMIDS  •  {source}/{mode}{paused}"

    def update(self, dt):
        if self.paused:
            return
        dt = min(float(dt), 0.05)
        now = time.time() - self.start_time
        mc.update(dt, now)
        state = director.update(dt, now, audio_engine.snapshot())
        self.draw_distance += self.draw_speed * dt * (0.35 + 1.65 * state.route_energy)
        self._emit_reactive_particles(state, dt)
        self._update_hud()

    def _emit_reactive_particles(self, state, dt):
        if not director.active or not len(state.particle_rate):
            return
        if len(self.particle_accumulator) != len(state.particle_rate):
            self.particle_accumulator = np.zeros(len(state.particle_rate))
        self.particle_accumulator += state.particle_rate * dt
        ready = np.flatnonzero(self.particle_accumulator >= 1.0)
        for index in ready[:12]:
            count = min(4, int(self.particle_accumulator[index]))
            self.particle_accumulator[index] -= count
            path = self._display_path(int(index))
            origin = path.mean(axis=0) if len(path) else director.topology.centers[index]
            mc.spawn_particles(count, origin)

    def _display_path(self, index):
        if director.active and index < director.topology.count:
            return apply_render_state(director.topology.home_paths[index], director.topology, index, director.output)
        return np.asarray(mc.pyramids[index].get_transformed_path(), dtype=float)

    def on_draw(self):
        glViewport(0, 0, self.width, self.height)
        glClearColor(0.008, 0.013, 0.021, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDisable(GL_CULL_FACE)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(58.0, float(self.width) / max(1.0, float(self.height)), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 8.5, self.camera_distance, 0.0, 0.4, 0.0, 0.0, 1.0, 0.0)
        now = time.time() - self.start_time
        glRotatef(now * 8.0, 0.0, 1.0, 0.0)
        self.draw_starfield()
        self.draw_floor_grid()
        self.draw_pyramids()
        self.draw_signal_trace()
        self.draw_particles()
        if self.cursor_visible:
            self.draw_cursor_marker()
        glDisable(GL_DEPTH_TEST)
        self.hud.draw()

    def draw_starfield(self):
        glPointSize(2.0)
        glBegin(GL_POINTS)
        for x, y, z, brightness in self.stars:
            glColor4f(brightness, brightness * 0.94, brightness * 0.78, brightness)
            glVertex3f(x, y, z)
        glEnd()

    def draw_floor_grid(self):
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for value in range(-8, 9):
            alpha = 0.22 if value == 0 else 0.08
            glColor4f(0.32, 0.62, 0.64, alpha)
            glVertex3f(value, -0.02, -8)
            glVertex3f(value, -0.02, 8)
            glVertex3f(-8, -0.02, value)
            glVertex3f(8, -0.02, value)
        glEnd()

    def draw_pyramids(self):
        state = director.output
        for index in range(len(mc.pyramids)):
            path = self._display_path(index)
            if len(path) < 2:
                continue
            if director.active:
                accent = _rainbow_color(state.accent_hue[index])
                mix = float(state.accent_mix[index])
                color = tuple(1.0 + (channel - 1.0) * mix for channel in accent)
                alpha = float(state.line_alpha[index])
                width = float(state.line_width[index])
            else:
                color, alpha, width = (1.0, 1.0, 1.0), 0.68, 1.5
            glLineWidth(width)
            glColor4f(*color, alpha)
            glBegin(GL_LINE_STRIP)
            for point in path:
                glVertex3f(*point)
            glEnd()

    def _trace_points(self):
        if director.active:
            if director.output.route_energy < 0.015:
                return []
            route = director.output.route[:96]
            points = []
            for index in route:
                if 0 <= index < len(mc.pyramids):
                    path = self._display_path(index)
                    if len(path):
                        if points:
                            points.append(path[0])
                        points.extend(path)
            return points
        points, _, _ = mc.build_global_path(close_loop=False, star_bridge=False)
        return points

    def draw_signal_trace(self):
        points = self._trace_points()
        if len(points) < 2:
            return
        lengths = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
        distances = np.concatenate(([0.0], np.cumsum(lengths)))
        total = float(distances[-1])
        if total <= 1e-9:
            return
        draw_to = self.draw_distance % total
        alpha = 0.35 + 0.65 * (director.output.route_energy if director.active else 1.0)
        glLineWidth(2.5 + (1.2 * director.output.route_energy if director.active else 0.5))
        glBegin(GL_LINE_STRIP)
        for index in range(len(points) - 1):
            if distances[index] > draw_to:
                break
            point_a, point_b = points[index], points[index + 1]
            glColor4f(*_rainbow_color(distances[index] / total), alpha)
            glVertex3f(*point_a)
            if distances[index + 1] <= draw_to:
                glColor4f(*_rainbow_color(distances[index + 1] / total), alpha)
                glVertex3f(*point_b)
            else:
                segment = max(1e-9, distances[index + 1] - distances[index])
                fraction = (draw_to - distances[index]) / segment
                midpoint = point_a + fraction * (point_b - point_a)
                glColor4f(*_rainbow_color(draw_to / total), alpha)
                glVertex3f(*midpoint)
                break
        glEnd()

    def draw_particles(self):
        if not mc.particles:
            return
        glPointSize(4.0)
        glBegin(GL_POINTS)
        for particle in mc.particles:
            alpha = max(0.0, min(1.0, particle.life / 2.0))
            glColor4f(*particle.color, alpha)
            glVertex3f(*particle.position)
        glEnd()

    def draw_cursor_marker(self):
        x, y, z = self.cursor_world
        glLineWidth(2.0)
        glColor4f(1.0, 0.72, 0.3, 0.7)
        glBegin(GL_LINES)
        glVertex3f(x - 0.25, y, z)
        glVertex3f(x + 0.25, y, z)
        glVertex3f(x, y, z - 0.25)
        glVertex3f(x, y, z + 0.25)
        glEnd()

    def _screen_to_world(self, x, y):
        return np.array([
            (x / max(1.0, self.width) - 0.5) * 13.0,
            0.35,
            -(y / max(1.0, self.height) - 0.5) * 10.0,
        ])

    def on_mouse_motion(self, x, y, dx, dy):
        del dx, dy
        self.cursor_world = self._screen_to_world(x, y)
        self.cursor_visible = True

    def on_mouse_press(self, x, y, button, modifiers):
        del modifiers
        if button == mouse.LEFT:
            self.cursor_world = self._screen_to_world(x, y)
            self.cursor_visible = True
            mc.handle_collision(self.cursor_world)

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        del x, y, scroll_x
        self.camera_distance = max(7.0, min(24.0, self.camera_distance - scroll_y * 0.8))

    def on_key_press(self, symbol, modifiers):
        del modifiers
        if symbol == key.SPACE:
            self.paused = not self.paused
        elif symbol == key.R:
            self.camera_distance = 14.0
            self.draw_distance = 0.0
        elif symbol == key.ESCAPE:
            audio_engine.shutdown()
            pyglet.app.exit()
        self._update_hud()

    def on_resize(self, width, height):
        super().on_resize(width, height)
        self.hud_brand.y = height - 30
        self.hud_scene.y = height - 55

    def on_close(self):
        pyglet.clock.unschedule(self.update)
        audio_engine.shutdown()
        pyglet.app.exit()


def run_gui():
    """Launch the private-by-default switchboard and companion stage."""
    global ui_window, visualization_window
    mc.configure_export(enabled=True, async_mode=False)
    mc.init_star_formation(subdivisions=1, core_radius=3.0, spike_height=2.4)
    mc.set_animation_mode(ANIMATION_WAVE_Y)
    for pyramid in mc.pyramids:
        pyramid.physics.wave_amplitude["y"] = knob_wave_value
    director.set_formation(mc.pyramids)
    ui_window = SwitchboardWindow()
    visualization_window = VisualizationWindow()
    screen = ui_window.screen
    gap = 18
    total_width = ui_window.width + visualization_window.width + gap
    if screen.width >= total_width + 40:
        left = max(20, (screen.width - total_width) // 2)
        bottom = max(24, (screen.height - max(ui_window.height, visualization_window.height)) // 2)
        ui_window.set_location(left, bottom)
        visualization_window.set_location(left + ui_window.width + gap, bottom + 54)
    else:
        ui_window.set_location(24, max(24, screen.height - ui_window.height - 50))
        visualization_window.set_location(max(45, screen.width - visualization_window.width - 30), 30)
    pyglet.app.run()


if __name__ == "__main__":
    run_gui()
