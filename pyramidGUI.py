#!/usr/bin/env python3
"""Rainbow Starburst desktop console and live OpenGL visualization.

The interface deliberately joins a tactile mid-century switchboard with a
restrained modern mixing desk.  The companion viewport renders the selected
pyramid formation, the animated rainbow trace, and interactive particle bursts.
"""

import colorsys
import math
import random
import time

import numpy as np
import pyglet
from pyglet import shapes
from pyglet.window import key, mouse

# PyOpenGL provides the compatibility-profile matrix and immediate-mode calls
# used by the original renderer.
from OpenGL.GL import *
from OpenGL.GLU import *

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


PALETTE = {
    "ink": (13, 18, 23),
    "panel": (27, 33, 38),
    "panel_raised": (39, 46, 52),
    "panel_hover": (51, 61, 68),
    "line": (76, 85, 91),
    "cream": (240, 229, 199),
    "muted": (163, 167, 156),
    "brass": (202, 164, 91),
    "amber": (255, 180, 73),
    "teal": (68, 202, 190),
    "red": (235, 92, 86),
    "blue": (94, 142, 236),
}

ARRANGEMENTS = ("Edge2Edge", "SpikeSphere", "Grid", "Star", "Globe")
PARTICLE_MODES = (PARTICLE_OFF, PARTICLE_LOW, PARTICLE_MEDIUM, PARTICLE_HEAVY)
ANIMATION_LABELS = ("WAVE", "SPIN", "PULSE", "NONE")
ANIMATION_VALUES = {
    "WAVE": ANIMATION_WAVE_Y,
    "SPIN": ANIMATION_SPIN,
    "PULSE": ANIMATION_PULSE,
    "NONE": "",
}


ui_window = None
visualization_window = None
mc = MasterController()

knob_wave_value = 0.8
knob_subdiv_value = 1.0
turntable_angle = 0.0

ui_batch = None
toggle_buttons = []
particle_buttons = []
animation_buttons = []
dial_knob_wave = None
dial_knob_subdiv = None
turntable = None
speaker_rect = None
led_lamp = None


def _rgba(rgb, alpha=255):
    return (*rgb, alpha)


def _rainbow_color(fraction, saturation=0.86, value=1.0):
    """Return a smooth RGB rainbow color for a normalized fraction."""
    hue = (fraction * 0.92) % 1.0
    return colorsys.hsv_to_rgb(hue, saturation, value)


class ToggleButton:
    """Tactile switch with active and hover states."""

    def __init__(
        self,
        x,
        y,
        width,
        height,
        text,
        on_press,
        batch,
        accent=None,
    ):
        self.x = x
        self.y = y
        self.w = width
        self.h = height
        self.label = text
        self.on_press_callback = on_press
        self.accent = accent or PALETTE["amber"]
        self.active = False
        self.hovered = False

        self.border = shapes.Rectangle(
            x - 2, y - 2, width + 4, height + 4,
            color=PALETTE["line"], batch=batch,
        )
        self.rect = shapes.Rectangle(
            x, y, width, height,
            color=PALETTE["panel_raised"], batch=batch,
        )
        self.signal = shapes.Rectangle(
            x + 10, y + 6, max(4, width - 20), 3,
            color=PALETTE["line"], batch=batch,
        )
        self.txt = pyglet.text.Label(
            text,
            x=x + width // 2,
            y=y + height // 2 + 3,
            anchor_x="center",
            anchor_y="center",
            font_name="Segoe UI",
            font_size=10,
            color=_rgba(PALETTE["cream"]),
            batch=batch,
        )

    def hit_test(self, mx, my):
        return self.x <= mx <= self.x + self.w and self.y <= my <= self.y + self.h

    def set_active(self, active):
        self.active = bool(active)
        self._refresh()

    def on_mouse_motion(self, mx, my):
        hovered = self.hit_test(mx, my)
        if hovered != self.hovered:
            self.hovered = hovered
            self._refresh()

    def on_mouse_press(self, mx, my, button, modifiers):
        if button == mouse.LEFT and self.hit_test(mx, my):
            self.on_press_callback()
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
    """Rotary control with a bounded value and a readable live value."""

    def __init__(
        self,
        x,
        y,
        radius,
        label,
        batch,
        on_drag,
        minimum=0.0,
        maximum=1.0,
        value=0.5,
        step=None,
        value_format="{:.1f}",
    ):
        self.x = x
        self.y = y
        self.r = radius
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.step = step
        self.value_format = value_format
        self.value = float(value)
        self.on_drag = on_drag
        self.dragging = False

        self.halo = shapes.Circle(x, y, radius + 5, color=PALETTE["line"], batch=batch)
        self.bg = shapes.Circle(x, y, radius, color=PALETTE["panel_raised"], batch=batch)
        self.cap = shapes.Circle(x, y, max(7, radius - 13), color=PALETTE["panel"], batch=batch)
        self.indicator = shapes.Line(
            x, y, x, y + radius - 7,
            thickness=4, color=PALETTE["amber"], batch=batch,
        )
        self.lbl = pyglet.text.Label(
            label.upper(),
            x=x,
            y=y + radius + 24,
            anchor_x="center",
            anchor_y="center",
            font_name="Segoe UI",
            font_size=9,
            color=_rgba(PALETTE["muted"]),
            batch=batch,
        )
        self.value_lbl = pyglet.text.Label(
            "",
            x=x,
            y=y - radius - 19,
            anchor_x="center",
            anchor_y="center",
            font_name="Consolas",
            font_size=10,
            color=_rgba(PALETTE["cream"]),
            batch=batch,
        )
        self.set_value(value, notify=False)

    def hit_test(self, mx, my):
        return math.hypot(mx - self.x, my - self.y) <= self.r + 5

    def on_mouse_press(self, mx, my, button, modifiers):
        if button == mouse.LEFT and self.hit_test(mx, my):
            self.dragging = True
            self._set_from_pointer(mx, my)
            return True
        return False

    def on_mouse_drag(self, mx, my, dx, dy, buttons, modifiers):
        if self.dragging and (buttons & mouse.LEFT):
            self._set_from_pointer(mx, my)

    def on_mouse_release(self, mx, my, button, modifiers):
        if self.dragging and button == mouse.LEFT:
            self.dragging = False

    def set_value(self, value, notify=True):
        value = max(self.minimum, min(self.maximum, float(value)))
        if self.step:
            value = round(value / self.step) * self.step
        self.value = value
        fraction = 0.0 if self.maximum == self.minimum else (
            (value - self.minimum) / (self.maximum - self.minimum)
        )
        angle = math.radians(225.0 + fraction * 270.0)
        length = self.r - 8
        self.indicator.x2 = self.x + math.cos(angle) * length
        self.indicator.y2 = self.y + math.sin(angle) * length
        self.value_lbl.text = self.value_format.format(value)
        if notify:
            self.on_drag(value)

    def _set_from_pointer(self, mx, my):
        angle = math.degrees(math.atan2(my - self.y, mx - self.x)) % 360.0
        relative = (angle - 225.0) % 360.0
        if relative > 270.0:
            # The unused 90-degree arc is the knob's hard stop.  Snap to the
            # nearest endpoint so dragging through it remains predictable.
            relative = 0.0 if relative > 315.0 else 270.0
        fraction = relative / 270.0
        self.set_value(self.minimum + fraction * (self.maximum - self.minimum))


class Turntable:
    """Large tactile disc controlling the globe apex offset."""

    def __init__(self, x, y, radius, label, on_spin, batch):
        self.x = x
        self.y = y
        self.r = radius
        self.angle = 0.0
        self.dragging = False
        self.on_spin = on_spin

        self.outer = shapes.Circle(x, y, radius + 6, color=PALETTE["brass"], batch=batch)
        self.bg = shapes.Circle(x, y, radius, color=(28, 37, 47), batch=batch)
        self.groove_a = shapes.Circle(x, y, radius - 10, color=(40, 50, 61), batch=batch)
        self.groove_b = shapes.Circle(x, y, radius - 17, color=(23, 30, 38), batch=batch)
        self.hub = shapes.Circle(x, y, 8, color=PALETTE["cream"], batch=batch)
        self.indicator = shapes.Line(
            x, y, x + radius - 12, y,
            thickness=3, color=PALETTE["amber"], batch=batch,
        )
        self.lbl = pyglet.text.Label(
            label.upper(),
            x=x,
            y=y + radius + 25,
            anchor_x="center",
            anchor_y="center",
            font_name="Segoe UI",
            font_size=9,
            color=_rgba(PALETTE["muted"]),
            batch=batch,
        )
        self.value_lbl = pyglet.text.Label(
            "000°",
            x=x,
            y=y - radius - 19,
            anchor_x="center",
            anchor_y="center",
            font_name="Consolas",
            font_size=10,
            color=_rgba(PALETTE["cream"]),
            batch=batch,
        )

    def hit_test(self, mx, my):
        return math.hypot(mx - self.x, my - self.y) <= self.r + 6

    def on_mouse_press(self, mx, my, button, modifiers):
        if button == mouse.LEFT and self.hit_test(mx, my):
            self.dragging = True
            self._set_from_pointer(mx, my)
            return True
        return False

    def on_mouse_drag(self, mx, my, dx, dy, buttons, modifiers):
        if self.dragging and (buttons & mouse.LEFT):
            self._set_from_pointer(mx, my)

    def on_mouse_release(self, mx, my, button, modifiers):
        if self.dragging and button == mouse.LEFT:
            self.dragging = False

    def _set_from_pointer(self, mx, my):
        self.angle = math.degrees(math.atan2(my - self.y, mx - self.x)) % 360.0
        radians = math.radians(self.angle)
        length = self.r - 12
        self.indicator.x2 = self.x + math.cos(radians) * length
        self.indicator.y2 = self.y + math.sin(radians) * length
        self.value_lbl.text = f"{int(round(self.angle)) % 360:03d}°"
        self.on_spin(self.angle)


class SwitchboardWindow(pyglet.window.Window):
    """Primary working surface for arrangements, signals, and motion."""

    def __init__(self, width=760, height=620, title="Rainbow Starburst — Switchboard"):
        super().__init__(width, height, title, resizable=False, vsync=True)

        global ui_batch, speaker_rect, led_lamp
        global dial_knob_wave, dial_knob_subdiv, turntable

        ui_batch = pyglet.graphics.Batch()
        toggle_buttons.clear()
        particle_buttons.clear()
        animation_buttons.clear()

        self.active_arrangement = "Star"
        self.active_particle = PARTICLE_OFF
        self.active_animation = "WAVE"
        self.status_expires = 0.0
        self.static_labels = []
        self.decoration_shapes = []

        self.background = shapes.Rectangle(
            0, 0, width, height, color=PALETTE["ink"], batch=ui_batch,
        )
        self.faceplate = shapes.Rectangle(
            20, 18, width - 40, height - 36,
            color=PALETTE["panel"], batch=ui_batch,
        )
        self.header_rule = shapes.Rectangle(
            40, height - 98, width - 80, 2,
            color=PALETTE["brass"], batch=ui_batch,
        )

        self._label(
            "RAINBOW / STARBURST", 40, height - 48,
            22, PALETTE["cream"], anchor_y="center",
        )
        self._label(
            "PYRAMID SIGNAL CONSOLE  •  RS–01", 42, height - 76,
            9, PALETTE["muted"], anchor_y="center",
        )
        self.live_dot = shapes.Circle(
            width - 138, height - 57, 6,
            color=PALETTE["teal"], batch=ui_batch,
        )
        self._label(
            "SYSTEM LIVE", width - 122, height - 57,
            9, PALETTE["teal"], anchor_y="center",
        )

        self._section_label("01  FORMATION ROUTING", 40, height - 132)
        button_width = 120
        gap = 14
        x_start = 40
        y_arrangements = height - 195
        for index, label in enumerate(ARRANGEMENTS):
            button = ToggleButton(
                x=x_start + index * (button_width + gap),
                y=y_arrangements,
                width=button_width,
                height=42,
                text=label.upper(),
                on_press=lambda value=label: self.on_arrangement_pressed(value),
                batch=ui_batch,
                accent=PALETTE["amber"],
            )
            toggle_buttons.append(button)

        self._section_label("02  PARTICLE GAIN", 40, height - 240)
        self._section_label("03  MOTION PROGRAM", 392, height - 240)
        compact_width = 72
        compact_gap = 10
        y_modes = height - 302
        for index, mode in enumerate(PARTICLE_MODES):
            button = ToggleButton(
                x=40 + index * (compact_width + compact_gap),
                y=y_modes,
                width=compact_width,
                height=40,
                text=mode,
                on_press=lambda value=mode: self.on_particle_mode_pressed(value),
                batch=ui_batch,
                accent=PALETTE["teal"],
            )
            particle_buttons.append(button)

        for index, label in enumerate(ANIMATION_LABELS):
            button = ToggleButton(
                x=392 + index * (compact_width + compact_gap),
                y=y_modes,
                width=compact_width,
                height=40,
                text=label,
                on_press=lambda value=label: self.on_animation_pressed(value),
                batch=ui_batch,
                accent=PALETTE["red"] if label == "SPIN" else PALETTE["blue"],
            )
            animation_buttons.append(button)

        self.deck_rule = shapes.Rectangle(
            40, 244, width - 80, 1, color=PALETTE["line"], batch=ui_batch,
        )
        self._section_label("04  CONTROL DECK", 40, 226)

        dial_knob_wave = Knob(
            x=105,
            y=128,
            radius=42,
            label="Wave amplitude",
            batch=ui_batch,
            on_drag=self.on_knob_wave_drag,
            minimum=0.0,
            maximum=2.0,
            value=knob_wave_value,
            step=0.05,
            value_format="{:.2f}",
        )
        dial_knob_subdiv = Knob(
            x=246,
            y=128,
            radius=42,
            label="Globe detail",
            batch=ui_batch,
            on_drag=self.on_knob_subdiv_drag,
            minimum=0.0,
            maximum=float(MAX_GLOBE_SUBDIVISIONS),
            value=knob_subdiv_value,
            step=1.0,
            value_format="{:.0f}",
        )
        turntable = Turntable(
            x=408,
            y=126,
            radius=54,
            label="Apex offset",
            on_spin=self.on_turntable_spin,
            batch=ui_batch,
        )

        speaker_rect = shapes.Rectangle(
            514, 69, 198, 112, color=(20, 25, 29), batch=ui_batch,
        )
        self.speaker_border = shapes.Rectangle(
            510, 65, 206, 120, color=PALETTE["line"], batch=ui_batch,
        )
        # Re-create the inner rectangle after the border so it stays visually inset.
        speaker_rect = shapes.Rectangle(
            514, 69, 198, 112, color=(20, 25, 29), batch=ui_batch,
        )
        for index in range(10):
            self.decoration_shapes.append(shapes.Rectangle(
                526,
                81 + index * 9,
                174,
                2,
                color=(54, 62, 66),
                batch=ui_batch,
            ))
        self._label(
            "SIGNAL MONITOR", 613, 198,
            9, PALETTE["muted"], anchor_x="center", anchor_y="center",
        )
        self.led_halo = shapes.Circle(
            613, 211, 10, color=(54, 62, 66), batch=ui_batch,
        )
        led_lamp = shapes.Circle(
            613, 211, 5, color=PALETTE["teal"], batch=ui_batch,
        )

        self.status_label = pyglet.text.Label(
            "",
            x=40,
            y=36,
            anchor_x="left",
            anchor_y="center",
            font_name="Consolas",
            font_size=9,
            color=_rgba(PALETTE["muted"]),
            batch=ui_batch,
        )
        self.help_label = pyglet.text.Label(
            "1–5 FORMATIONS  •  CLICK VIEWPORT FOR BURSTS",
            x=width - 40,
            y=36,
            anchor_x="right",
            anchor_y="center",
            font_name="Segoe UI",
            font_size=8,
            color=_rgba(PALETTE["muted"]),
            batch=ui_batch,
        )

        self._sync_active_states()
        self.set_status("STAR FORMATION ROUTED  •  WAVE PROGRAM ACTIVE")

    def _label(
        self,
        text,
        x,
        y,
        font_size,
        color,
        anchor_x="left",
        anchor_y="baseline",
    ):
        label = pyglet.text.Label(
            text,
            x=x,
            y=y,
            anchor_x=anchor_x,
            anchor_y=anchor_y,
            font_name="Segoe UI",
            font_size=font_size,
            color=_rgba(color),
            batch=ui_batch,
        )
        self.static_labels.append(label)
        return label

    def _section_label(self, text, x, y):
        return self._label(text, x, y, 9, PALETTE["brass"])

    def _sync_active_states(self):
        for button, value in zip(toggle_buttons, ARRANGEMENTS):
            button.set_active(value == self.active_arrangement)
        for button, value in zip(particle_buttons, PARTICLE_MODES):
            button.set_active(value == self.active_particle)
        for button, value in zip(animation_buttons, ANIMATION_LABELS):
            button.set_active(value == self.active_animation)

    def set_status(self, message, duration=4.0):
        self.status_label.text = message
        self.status_expires = time.time() + duration

    def on_draw(self):
        glClearColor(*(channel / 255.0 for channel in PALETTE["ink"]), 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        ui_batch.draw()

    def on_mouse_motion(self, x, y, dx, dy):
        for button in toggle_buttons + particle_buttons + animation_buttons:
            button.on_mouse_motion(x, y)

    def on_mouse_press(self, x, y, button, modifiers):
        for control in toggle_buttons + particle_buttons + animation_buttons:
            if control.on_mouse_press(x, y, button, modifiers):
                return
        if dial_knob_wave.on_mouse_press(x, y, button, modifiers):
            return
        if dial_knob_subdiv.on_mouse_press(x, y, button, modifiers):
            return
        turntable.on_mouse_press(x, y, button, modifiers)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        dial_knob_wave.on_mouse_drag(x, y, dx, dy, buttons, modifiers)
        dial_knob_subdiv.on_mouse_drag(x, y, dx, dy, buttons, modifiers)
        turntable.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

    def on_mouse_release(self, x, y, button, modifiers):
        dial_knob_wave.on_mouse_release(x, y, button, modifiers)
        dial_knob_subdiv.on_mouse_release(x, y, button, modifiers)
        turntable.on_mouse_release(x, y, button, modifiers)

    def on_key_press(self, symbol, modifiers):
        arrangement_keys = {
            key._1: "Edge2Edge",
            key._2: "SpikeSphere",
            key._3: "Grid",
            key._4: "Star",
            key._5: "Globe",
        }
        if symbol in arrangement_keys:
            self.on_arrangement_pressed(arrangement_keys[symbol])
        elif symbol == key.ESCAPE:
            pyglet.app.exit()

    def on_close(self):
        pyglet.app.exit()

    def on_arrangement_pressed(self, label):
        if label == "Edge2Edge":
            mc.init_edge_to_edge_pyramids(count=7)
        elif label == "SpikeSphere":
            mc.init_spike_sphere_pyramids(count=24, sphere_radius=4.0)
        elif label == "Grid":
            mc.init_grid_pyramids(rows=4, cols=5, spacing_x=1.8, spacing_z=1.8)
        elif label == "Star":
            mc.init_star_formation(subdivisions=1, core_radius=3.0, spike_height=2.4)
        elif label == "Globe":
            subdivisions = int(round(knob_subdiv_value))
            offset = 0.18 + (turntable_angle % 360.0) / 360.0 * 0.7
            mc.init_globe_icosahedron(
                subdivisions=subdivisions,
                apex_offset=offset,
                base_scale=4.0,
            )

        self.active_arrangement = label
        mc.set_animation_mode(ANIMATION_VALUES[self.active_animation])
        self._apply_wave_value()
        self._sync_active_states()
        self.set_status(f"{label.upper()} ROUTED  •  {len(mc.pyramids)} PYRAMIDS")
        if visualization_window is not None:
            visualization_window.set_scene(label)

    def on_particle_mode_pressed(self, mode):
        self.active_particle = mode
        mc.set_particle_mode(mode)
        self._sync_active_states()
        message = "PARTICLE GAIN MUTED" if mode == PARTICLE_OFF else (
            f"PARTICLE GAIN {mode}  •  CLICK THE 3D VIEWPORT TO BURST"
        )
        self.set_status(message)

    def on_animation_pressed(self, label):
        self.active_animation = label
        mc.set_animation_mode(ANIMATION_VALUES[label])
        self._apply_wave_value()
        self._sync_active_states()

        lamp_colors = {
            "WAVE": PALETTE["teal"],
            "SPIN": PALETTE["red"],
            "PULSE": PALETTE["blue"],
            "NONE": (38, 44, 47),
        }
        led_lamp.color = lamp_colors[label]
        self.led_halo.color = lamp_colors[label] if label != "NONE" else PALETTE["line"]
        self.set_status(f"MOTION PROGRAM {label}")

    def _apply_wave_value(self):
        if self.active_animation not in ("WAVE", "PULSE"):
            return
        multiplier = 1.65 if self.active_animation == "PULSE" else 1.0
        for pyramid in mc.pyramids:
            if pyramid.physics.wave_axis_enable["y"]:
                pyramid.physics.wave_amplitude["y"] = knob_wave_value * multiplier

    def on_knob_wave_drag(self, new_value):
        global knob_wave_value
        knob_wave_value = float(new_value)
        self._apply_wave_value()
        self.set_status(f"WAVE AMPLITUDE  {knob_wave_value:.2f}", duration=2.0)

    def on_knob_subdiv_drag(self, new_value):
        global knob_subdiv_value
        knob_subdiv_value = float(int(round(new_value)))
        faces = 20 * (4 ** int(knob_subdiv_value))
        self.set_status(
            f"GLOBE DETAIL  {int(knob_subdiv_value)}  •  {faces} FACES ON NEXT ROUTE",
            duration=2.0,
        )

    def on_turntable_spin(self, angle):
        global turntable_angle
        turntable_angle = float(angle)
        offset = 0.18 + (turntable_angle % 360.0) / 360.0 * 0.7
        self.set_status(f"GLOBE APEX OFFSET  {offset:.2f}", duration=2.0)


class VisualizationWindow(pyglet.window.Window):
    """Live 3D stage for the selected formation and its signal trace."""

    def __init__(self, width=900, height=650, title="Rainbow Starburst — 3D Signal"):
        super().__init__(width, height, title, resizable=True, vsync=True)
        self.set_minimum_size(560, 420)
        self.start_time = time.time()
        self.draw_dist = 0.0
        self.drawing_forward = True
        self.draw_speed = 8.0
        self.camera_distance = 14.0
        self.scene_name = "Star"
        self.paused = False
        self.cursor_world = np.zeros(3, dtype=float)
        self.cursor_visible = False

        rng = random.Random(86)
        self.stars = [
            (
                rng.uniform(-14.0, 14.0),
                rng.uniform(-6.0, 11.0),
                rng.uniform(-12.0, -2.0),
                rng.uniform(0.25, 0.8),
            )
            for _ in range(120)
        ]

        self.hud_batch = pyglet.graphics.Batch()
        self.hud_brand = pyglet.text.Label(
            "RAINBOW / STARBURST",
            x=26,
            y=height - 30,
            anchor_x="left",
            anchor_y="top",
            font_name="Segoe UI",
            font_size=13,
            color=_rgba(PALETTE["cream"]),
            batch=self.hud_batch,
        )
        self.hud_scene = pyglet.text.Label(
            "",
            x=26,
            y=height - 55,
            anchor_x="left",
            anchor_y="top",
            font_name="Consolas",
            font_size=9,
            color=_rgba(PALETTE["teal"]),
            batch=self.hud_batch,
        )
        self.hud_help = pyglet.text.Label(
            "CLICK: PARTICLE BURST   •   SCROLL: CAMERA   •   SPACE: PAUSE",
            x=26,
            y=22,
            anchor_x="left",
            anchor_y="bottom",
            font_name="Segoe UI",
            font_size=8,
            color=_rgba(PALETTE["muted"]),
            batch=self.hud_batch,
        )
        self._update_hud()
        pyglet.clock.schedule_interval(self.update, 1.0 / 60.0)

    def set_scene(self, name):
        self.scene_name = name
        self.draw_dist = 0.0
        self.drawing_forward = True
        self._update_hud()

    def _update_hud(self):
        paused = "  •  PAUSED" if self.paused else ""
        self.hud_scene.text = (
            f"{self.scene_name.upper()}  •  {len(mc.pyramids)} PYRAMIDS"
            f"  •  PARTICLES {mc.current_particle_mode}{paused}"
        )

    def update(self, dt):
        if self.paused:
            return
        dt = min(dt, 0.05)
        current_time = time.time() - self.start_time
        mc.update(dt, current_time)
        self.draw_dist += self.draw_speed * dt * (1.0 if self.drawing_forward else -1.0)
        self._update_hud()

    def on_draw(self):
        glViewport(0, 0, self.width, self.height)
        glClearColor(0.012, 0.018, 0.028, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDisable(GL_CULL_FACE)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = float(self.width) / max(1.0, float(self.height))
        gluPerspective(58.0, aspect, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 8.5, self.camera_distance, 0.0, 0.4, 0.0, 0.0, 1.0, 0.0)

        now = time.time() - self.start_time
        glRotatef(now * 8.0, 0.0, 1.0, 0.0)

        self.draw_starfield()
        self.draw_floor_grid()
        self.draw_pyramids()
        self.draw_rainbow_path()
        self.draw_particles()
        if self.cursor_visible:
            self.draw_cursor_marker()

        glDisable(GL_DEPTH_TEST)
        self.hud_batch.draw()

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
        glLineWidth(1.5)
        glColor4f(1.0, 1.0, 1.0, 0.68)
        for pyramid in mc.pyramids:
            path = pyramid.get_transformed_path()
            if len(path) < 2:
                continue
            glBegin(GL_LINE_STRIP)
            for point in path:
                glVertex3f(float(point[0]), float(point[1]), float(point[2]))
            glEnd()

    def draw_rainbow_path(self):
        points, total_length, distances = mc.build_global_path(
            close_loop=False,
            star_bridge=False,
        )
        if len(points) < 2 or total_length <= 1e-9:
            return

        if self.draw_dist > total_length:
            self.draw_dist = total_length
            self.drawing_forward = False
        elif self.draw_dist < 0.0:
            self.draw_dist = 0.0
            self.drawing_forward = True

        glLineWidth(3.0)
        glBegin(GL_LINE_STRIP)
        for index in range(len(points) - 1):
            segment_start = distances[index]
            segment_end = distances[index + 1]
            if segment_start > self.draw_dist:
                break
            point_a = points[index]
            point_b = points[index + 1]

            fraction_a = segment_start / total_length
            glColor3f(*_rainbow_color(fraction_a))
            glVertex3f(float(point_a[0]), float(point_a[1]), float(point_a[2]))

            if segment_end <= self.draw_dist:
                fraction_b = segment_end / total_length
                glColor3f(*_rainbow_color(fraction_b))
                glVertex3f(float(point_b[0]), float(point_b[1]), float(point_b[2]))
                continue

            segment_length = segment_end - segment_start
            if segment_length > 1e-9:
                alpha = (self.draw_dist - segment_start) / segment_length
                midpoint = point_a + alpha * (point_b - point_a)
                glColor3f(*_rainbow_color(self.draw_dist / total_length))
                glVertex3f(float(midpoint[0]), float(midpoint[1]), float(midpoint[2]))
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
        normalized_x = (x / max(1.0, self.width) - 0.5) * 13.0
        normalized_z = (y / max(1.0, self.height) - 0.5) * 10.0
        return np.array([normalized_x, 0.35, -normalized_z], dtype=float)

    def on_mouse_motion(self, x, y, dx, dy):
        self.cursor_world = self._screen_to_world(x, y)
        self.cursor_visible = True

    def on_mouse_press(self, x, y, button, modifiers):
        if button == mouse.LEFT:
            self.cursor_world = self._screen_to_world(x, y)
            self.cursor_visible = True
            mc.handle_collision(self.cursor_world)
            self._update_hud()

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.camera_distance = max(7.0, min(24.0, self.camera_distance - scroll_y * 0.8))

    def on_key_press(self, symbol, modifiers):
        if symbol == key.SPACE:
            self.paused = not self.paused
            self._update_hud()
        elif symbol == key.R:
            self.camera_distance = 14.0
            self.draw_dist = 0.0
            self.drawing_forward = True
        elif symbol == key.ESCAPE:
            pyglet.app.exit()

    def on_resize(self, width, height):
        super().on_resize(width, height)
        self.hud_brand.y = height - 30
        self.hud_scene.y = height - 55

    def on_close(self):
        pyglet.clock.unschedule(self.update)
        pyglet.app.exit()


def run_gui():
    """Launch the switchboard and its companion visualization window."""
    global ui_window, visualization_window

    # A scene switch is also a programming operation: when it returns, the
    # exported files must already match the active formation exactly.
    mc.configure_export(enabled=True, async_mode=False)
    mc.init_star_formation(subdivisions=1, core_radius=3.0, spike_height=2.4)
    mc.set_animation_mode(ANIMATION_WAVE_Y)
    for pyramid in mc.pyramids:
        pyramid.physics.wave_amplitude["y"] = knob_wave_value

    ui_window = SwitchboardWindow()
    visualization_window = VisualizationWindow()

    screen = ui_window.screen
    gap = 20
    total_width = ui_window.width + visualization_window.width + gap
    if screen.width >= total_width + 60:
        left = max(30, (screen.width - total_width) // 2)
        bottom = max(40, (screen.height - max(ui_window.height, visualization_window.height)) // 2)
        ui_window.set_location(left, bottom + 15)
        visualization_window.set_location(left + ui_window.width + gap, bottom)
    else:
        ui_window.set_location(30, max(30, screen.height - ui_window.height - 60))
        visualization_window.set_location(
            max(60, screen.width - visualization_window.width - 40),
            40,
        )

    pyglet.app.run()


if __name__ == "__main__":
    run_gui()
