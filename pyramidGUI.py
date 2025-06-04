Don Juan <chazkraiza@gmail.com>
	
Wed, Apr 2, 10:32 AM
	
to me
#!/usr/bin/env python3
"""
gui.py - A more refined, playful GUI bridging a 1950s phone switchboard aesthetic
         with a modern audio mixing console, controlling the MasterController.

Enhancements:
1) TWO knobs:
   - Knob #1 => wave amplitude
   - Knob #2 => globe subdivisions
2) Particle mode toggles: OFF, LOW, MEDIUM, HEAVY
3) Wave/Spin/Pulse toggles for animation modes
4) Extra shapes for theming:
   - "Speaker grill" rectangle
   - "LED lamp" that glows if wave/spin is active
5) Two windows:
   - Switchboard console (UI)
   - 3D Visualization
6) Uses PyOpenGL calls for GL functions to avoid missing 'glMatrixMode' in pyglet.gl.

Requires:
 - pyglet
 - PyOpenGL
 - numpy
 - master_controller.py and pyramid.py in the same folder
"""

import pyglet
from pyglet.window import key, mouse
from pyglet import shapes
import math
import time
import numpy as np

# --- Importing PyOpenGL to fix the glMatrixMode issue ---
from OpenGL.GL import *
from OpenGL.GLU import *

from master_controller import (
    MasterController, 
    PARTICLE_OFF, PARTICLE_LOW, PARTICLE_MEDIUM, PARTICLE_HEAVY,
    # Possibly also ANIMATION_WAVE_Y, ANIMATION_SPIN, ANIMATION_PULSE
)

###############################################################################
# GLOBAL / SHARED
###############################################################################
ui_window = None
visualization_window = None

mc = MasterController()

# KNOB #1 => wave amplitude
knob_wave_value = 0.5
# KNOB #2 => globe subdivisions
knob_subdiv_value = 1.0

turntable_angle = 0.0

ui_batch         = pyglet.graphics.Batch()
toggle_buttons   = []
particle_buttons = []
animation_buttons= []
dial_knob_wave   = None
dial_knob_subdiv = None
turntable        = None

# A "speaker grill" or decorative shape
speaker_rect     = None
# A "LED lamp" that glows if wave/spin/pulse is active
led_lamp         = None

###############################################################################
# Switchboard Window
###############################################################################
class SwitchboardWindow(pyglet.window.Window):
    def __init__(self, width, height, title):
        super().__init__(width, height, title, resizable=False)
        self.set_location(100,100)
        
        # Arrangement toggles
        arrangement_labels = ["Edge2Edge", "SpikeSphere", "Grid", "Star", "Globe"]
        x_start = 50
        y_start = self.height - 70
        spacing= 70
        for i, lbl in enumerate(arrangement_labels):
            btn = ToggleButton(
                x=x_start+ i* spacing,
                y=y_start,
                width=60,
                height=30,
                text=lbl,
                on_press=lambda l=lbl: self.on_arrangement_pressed(l),
                batch=ui_batch
            )
            toggle_buttons.append(btn)

        # Particle mode toggles
        pmodes = [PARTICLE_OFF, PARTICLE_LOW, PARTICLE_MEDIUM, PARTICLE_HEAVY]
        y2     = y_start - 50
        for i, m in enumerate(pmodes):
            btn = ToggleButton(
                x=x_start+ i* spacing,
                y=y2,
                width=60,
                height=30,
                text=m,
                on_press=lambda mode=m: self.on_particle_mode_pressed(mode),
                batch=ui_batch
            )
            particle_buttons.append(btn)

        # Animation toggles (Wave, Spin, Pulse, None)
        anim_labels = ["WAVE", "SPIN", "PULSE", "NONE"]
        y3          = y2 - 50
        for i, lbl in enumerate(anim_labels):
            btn = ToggleButton(
                x=x_start+ i* spacing,
                y=y3,
                width=60,
                height=30,
                text=lbl,
                on_press=lambda l=lbl: self.on_animation_pressed(l),
                batch=ui_batch
            )
            animation_buttons.append(btn)

        # A decorative "speaker grill" rectangle
        global speaker_rect
        speaker_rect = shapes.Rectangle(
            x= 300,
            y= 20,
            width= 180,
            height= 100,
            color=(70,70,70),
            batch= ui_batch
        )

        # "LED lamp" circle
        global led_lamp
        led_lamp= shapes.Circle(
            x= speaker_rect.x+ speaker_rect.width//2,
            y= speaker_rect.y+ speaker_rect.height+ 20,
            radius= 10,
            color=(0,0,0),
            batch= ui_batch
        )

        # Knob #1 => wave amplitude
        global dial_knob_wave
        dial_knob_wave= Knob(
            x=100, y=160, radius=30,
            label="WaveAmp", 
            batch= ui_batch,
            on_drag=self.on_knob_wave_drag
        )

        # Knob #2 => globe subdivisions
        global dial_knob_subdiv
        dial_knob_subdiv= Knob(
            x=220, y=160, radius=30,
            label="GlobeSubdiv",
            batch= ui_batch,
            on_drag=self.on_knob_subdiv_drag
        )

        # Turntable => apex offset or anything
        global turntable
        turntable= Turntable(
            x= 380, y= 200, radius=50,
            label="Turntable",
            on_spin=self.on_turntable_spin,
            batch= ui_batch
        )

    def on_draw(self):
        self.clear()
        # We'll just do a quick background color
        glClearColor(0.25, 0.25, 0.25, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        ui_batch.draw()

    # ARRANGEMENTS
    def on_arrangement_pressed(self, label):
        if label=="Edge2Edge":
            mc.init_edge_to_edge_pyramids(count=5)
        elif label=="SpikeSphere":
            mc.init_spike_sphere_pyramids(count=10, sphere_radius=5.0)
        elif label=="Grid":
            mc.init_grid_pyramids(rows=3, cols=4, spacing_x=2.0, spacing_z=2.0)
        elif label=="Star":
            mc.init_star_formation(count=6, radius=5.0)
        elif label=="Globe":
            global knob_subdiv_value
            subdiv = int(knob_subdiv_value)
            if subdiv<0: subdiv=0
            global turntable_angle
            offset = 0.1 + (turntable_angle %360)/360.0
            mc.init_globe_icosahedron(subdivisions=subdiv, apex_offset= offset, base_scale=1.0)

    # PARTICLE MODE
    def on_particle_mode_pressed(self, mode):
        mc.set_particle_mode(mode)
        print(f"[Particle Mode] => {mode}")

    # ANIMATION
    def on_animation_pressed(self, lbl):
        from master_controller import (ANIMATION_WAVE_Y, ANIMATION_SPIN, ANIMATION_PULSE)
        if lbl=="WAVE":
            mc.set_animation_mode(ANIMATION_WAVE_Y)
            led_lamp.color= (0,255,0)   # green LED
        elif lbl=="SPIN":
            mc.set_animation_mode(ANIMATION_SPIN)
            led_lamp.color= (255,0,0)   # red LED
        elif lbl=="PULSE":
            mc.set_animation_mode(ANIMATION_PULSE)
            led_lamp.color= (0,0,255)   # blue LED
        else:
            mc.set_animation_mode("")
            led_lamp.color= (0,0,0)     # off

    # KNOBS
    def on_knob_wave_drag(self, new_value):
        """
        wave amplitude real-time
        """
        global knob_wave_value
        knob_wave_value = new_value
        # apply to all pyramids if wave is enabled on y
        for p in mc.pyramids:
            if p.physics.wave_axis_enable["y"]:
                p.physics.wave_amplitude["y"] = new_value
        print(f"[KnobWave] => {knob_wave_value:.2f}")

    def on_knob_subdiv_drag(self, new_value):
        """
        sets globe subdivisions => can re-init if we want real-time
        """
        global knob_subdiv_value
        knob_subdiv_value= new_value
        print(f"[KnobSubdiv] => {knob_subdiv_value:.2f}")
        # optional auto-update:
        # subdiv= int(knob_subdiv_value)
        # offset= 0.1 + (turntable_angle%360)/360.0
        # mc.init_globe_icosahedron(subdivisions=subdiv, apex_offset= offset, base_scale=1.0)

    def on_turntable_spin(self, angle):
        global turntable_angle
        turntable_angle= angle
        print(f"[Turntable spin] => angle= {turntable_angle:.1f}")

###############################################################################
# Visualization Window
###############################################################################
class VisualizationWindow(pyglet.window.Window):
    def __init__(self, width, height, title):
        super().__init__(width, height, title, resizable=True)
        self.set_location(650,100)
        self.start_time = time.time()
        self.drawDist   = 0.0
        self.drawing_forward= True
        self.draw_speed = 2.0

        pyglet.clock.schedule_interval(self.update, 1/60.0)

    def update(self, dt):
        if self.drawing_forward:
            self.drawDist+= self.draw_speed* dt
        else:
            self.drawDist-= self.draw_speed* dt

    def on_draw(self):
        self.clear()
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect= float(self.width)/ self.height
        gluPerspective(60.0, aspect, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0,3,12, 0,0,0, 0,1,0)

        now= time.time()- self.start_time
        glRotatef(now*15.0, 0,1,0)

        # draw pyramids
        self.draw_pyramids()
        # bridging partial line
        self.draw_rainbow_path()

    def draw_pyramids(self):
        for p in mc.pyramids:
            tpath= p.get_transformed_path()
            if len(tpath)<2:
                continue
            glLineWidth(1.0)
            glColor3f(1,1,1)
            glBegin(GL_LINE_STRIP)
            for pt in tpath:
                glVertex3f(pt[0], pt[1], pt[2])
            glEnd()

    def draw_rainbow_path(self):
        points, total_len, dists= mc.build_global_path(close_loop=False, star_bridge=False)
        if len(points)<2:
            return
        if self.drawDist> total_len:
            self.drawDist= total_len
            self.drawing_forward= False
        elif self.drawDist<0:
            self.drawDist= 0
            self.drawing_forward= True

        glLineWidth(2.0)
        glBegin(GL_LINE_STRIP)
        def get_color(frac):
            hue= (frac*3.0)%1.0
            i= int(hue*6)
            f= hue*6- i
            q= 1- f
            if i==0: return (1, f, 0)
            if i==1: return (q, 1, 0)
            if i==2: return (0, 1, f)
            if i==3: return (0, q, 1)
            if i==4: return (f, 0, 1)
            return (1, 0, q)

        dist_clamped= self.drawDist
        for i in range(len(points)-1):
            seg_start= dists[i]
            seg_end  = dists[i+1]
            if seg_start> dist_clamped:
                break
            A= points[i]
            B= points[i+1]
            if seg_end< dist_clamped:
                fracA= seg_start/ total_len
                cA= get_color(fracA)
                glColor3f(*cA)
                glVertex3f(A[0],A[1],A[2])

                fracB= seg_end/ total_len
                cB= get_color(fracB)
                glColor3f(*cB)
                glVertex3f(B[0],B[1],B[2])
            else:
                alpha= (dist_clamped- seg_start)/(seg_end- seg_start)
                midP= A+ alpha*(B- A)
                fracA= seg_start/ total_len
                cA= get_color(fracA)
                glColor3f(*cA)
                glVertex3f(A[0], A[1], A[2])

                fracM= dist_clamped/ total_len
                cM= get_color(fracM)
                glColor3f(*cM)
                glVertex3f(midP[0], midP[1], midP[2])
                break
        glEnd()

###############################################################################
# UI CLASSES
###############################################################################
class ToggleButton:
    def __init__(self, x, y, width, height, text, on_press, batch):
        self.x = x
        self.y = y
        self.w = width
        self.h = height
        self.on_press_callback= on_press
        self.label= text

        self.rect= shapes.Rectangle(x, y, width, height, color=(50,50,50), batch=batch)
        self.txt= pyglet.text.Label(
            text, 
            x= x+ width//2,
            y= y+ height//2,
            anchor_x='center',
            anchor_y='center',
            font_size=8,
            batch=batch
        )

    def hit_test(self, mx, my):
        return (mx>=self.x and mx<= self.x+self.w and
                my>=self.y and my<= self.y+self.h)

    def on_mouse_press(self, mx, my, button, modifiers):
        if button== mouse.LEFT:
            if self.hit_test(mx,my):
                self.on_press_callback()

class Knob:
    """
    A simple rotary knob => range 0..10
    """
    def __init__(self, x, y, radius, label, batch, on_drag):
        self.x= x
        self.y= y
        self.r= radius
        self.value= 1.0
        self.on_drag= on_drag
        self.dragging= False
        self.bg= shapes.Circle(x, y, radius, color=(120,120,120), batch=batch)
        self.lbl= pyglet.text.Label(
            label,
            x=x,
            y=y+ radius+ 15,
            anchor_x='center',
            anchor_y='center',
            font_size=10,
            batch=batch
        )

    def hit_test(self, mx, my):
        dx= mx- self.x
        dy= my- self.y
        dist= math.sqrt(dx*dx+ dy*dy)
        return dist<= self.r

    def on_mouse_press(self, mx, my, button, modifiers):
        if button== mouse.LEFT:
            if self.hit_test(mx,my):
                self.dragging= True

    def on_mouse_drag(self, mx, my, dx, dy, buttons, modifiers):
        if self.dragging and (buttons & mouse.LEFT):
            angle= math.degrees(math.atan2(my- self.y, mx- self.x))
            if angle<0:
                angle+=360
            new_val= (angle/360.0)*10.0
            self.value= new_val
            self.on_drag(new_val)

    def on_mouse_release(self, mx, my, button, modifiers):
        if self.dragging and button== mouse.LEFT:
            self.dragging= False

class Turntable:
    """
    Spinning disc => angle => on_spin(angle)
    """
    def __init__(self, x, y, radius, label, on_spin, batch):
        self.x= x
        self.y= y
        self.r= radius
        self.angle= 0
        self.dragging= False
        self.on_spin= on_spin
        self.bg= shapes.Circle(x, y, radius, color=(80,80,200), batch=batch)
        self.lbl= pyglet.text.Label(
            label,
            x=x, y=y- radius-15,
            anchor_x='center', anchor_y='center',
            font_size=10,
            batch=batch
        )

    def hit_test(self, mx, my):
        dx= mx- self.x
        dy= my- self.y
        dist= math.sqrt(dx*dx+ dy*dy)
        return dist<= self.r

    def on_mouse_press(self, mx, my, button, modifiers):
        if button== mouse.LEFT:
            if self.hit_test(mx,my):
                self.dragging= True

    def on_mouse_drag(self, mx, my, dx, dy, buttons, modifiers):
        if self.dragging and (buttons & mouse.LEFT):
            angle= math.degrees(math.atan2(my- self.y, mx- self.x))
            if angle<0:
                angle+=360
            self.angle= angle
            self.on_spin(angle)

    def on_mouse_release(self, mx, my, button, modifiers):
        if self.dragging and button== mouse.LEFT:
            self.dragging= False

###############################################################################
def run_gui():
    global ui_window, visualization_window

    ui_window= SwitchboardWindow(width=600, height=500, title="Switchboard Console")
    visualization_window= VisualizationWindow(width=800, height=600, title="3D Visualization")

    @ui_window.event
    def on_mouse_press(x,y, button, modifiers):
        for t in toggle_buttons:
            t.on_mouse_press(x,y,button,modifiers)
        for t in particle_buttons:
            t.on_mouse_press(x,y,button,modifiers)
        for t in animation_buttons:
            t.on_mouse_press(x,y,button,modifiers)
        dial_knob_wave.on_mouse_press(x,y,button,modifiers)
        dial_knob_subdiv.on_mouse_press(x,y,button,modifiers)
        turntable.on_mouse_press(x,y,button,modifiers)

    @ui_window.event
    def on_mouse_drag(x,y, dx, dy, buttons, modifiers):
        dial_knob_wave.on_mouse_drag(x,y,dx,dy,buttons,modifiers)
        dial_knob_subdiv.on_mouse_drag(x,y,dx,dy,buttons,modifiers)
        turntable.on_mouse_drag(x,y,dx,dy,buttons,modifiers)

    @ui_window.event
    def on_mouse_release(x,y, button, modifiers):
        dial_knob_wave.on_mouse_release(x,y,button,modifiers)
        dial_knob_subdiv.on_mouse_release(x,y,button,modifiers)
        turntable.on_mouse_release(x,y,button,modifiers)

    pyglet.app.run()

if __name__=="__main__":
    run_gui()
