#!/usr/bin/env python3
"""
pyramid.py - A fully-featured pyramid template for your Rainbow Star Burst Project.

FEATURES:
1) Parametric Base Shape (num_corners, base_length, base_width) + apex_height
2) Per-corner offsets for local shape warping
3) Dynamic labeling via pyramid_id => labeling each corner, apex, base edges, apex edges
4) Full 3D Euler rotation (pitch, yaw, roll) + wave-based motion (X, Y, Z)
5) set_pyramid_id(new_id) => re-generate unique labels on the fly
6) Two built-in demo modes:
   - console_demo() => 5s wave/bounce test in console
   - gui_demo() => Pyglet+OpenGL window for real-time 3D viewing

Usage:
  python pyramid.py

By default, main() calls gui_demo(). If you prefer the console test, edit main().
"""

import math
import time
import numpy as np

# Pyglet + OpenGL imports (kept at module level to avoid syntax errors)
import pyglet
from pyglet.window import key
from OpenGL.GL import *
from OpenGL.GLU import *

###############################################################################
# LABELING
###############################################################################

def generate_pyramid_labels(pyramid_id, num_corners=4):
    """
    Generates unique labels for a pyramid-like shape, referencing 'pyramid_id'.
    Markers:
      corners: <pyramid_id>-1..N
      apex: <pyramid_id>-(num_corners+1)
      base edges: <pyramid_id>-AA.. (one per corner)
      apex edges: <pyramid_id>-a.. (one per corner)

    By default for square (num_corners=4):
      corners => corner1..4 => IDs: "X-1","X-2","X-3","X-4"
      apex => "X-5"
      base edges => "X-AA".."X-DD"
      apex edges => "X-a".."X-d"
    """
    base_edge_labels = ["AA","BB","CC","DD","EE","FF","GG","HH"]
    apex_edge_labels = ["a", "b", "c", "d","e","f","g","h"]  

    labels = {}

    # 1) Corner Labels
    for i in range(1, num_corners+1):
        labels[f"corner{i}"] = f"{pyramid_id}-{i}"

    # 2) Apex => <pyramid_id>-(num_corners+1)
    apex_index = num_corners + 1
    labels["apex"] = f"{pyramid_id}-{apex_index}"

    # 3) Base edges (AA.. or fallback)
    for i in range(num_corners):
        if i < len(base_edge_labels):
            edge_label = base_edge_labels[i]
        else:
            edge_label = f"edge{i}"
        labels[f"base_edge{i+1}"] = f"{pyramid_id}-{edge_label}"

    # 4) Apex edges (a.. or fallback)
    for i in range(num_corners):
        if i < len(apex_edge_labels):
            apex_edge_label = apex_edge_labels[i]
        else:
            apex_edge_label = f"apexEdge{i}"
        labels[f"apex_edge{i+1}"] = f"{pyramid_id}-{apex_edge_label}"

    return labels

###############################################################################
# PATH BUILDING
###############################################################################

def build_local_pyramid_path(num_corners=4,
                             base_length=1.0,
                             base_width=1.0,
                             apex_height=2.0):
    """
    Builds a local path for the pyramid, visiting edges exactly once in a single stroke:
      1) Base loop: c1->c2->...->cN->c1
      2) Sides: c1->apex->c2->apex->...->cN->apex->c1
    """
    path = []

    # Generate base corners
    corners = []
    if num_corners == 4:
        # Rectangular base if corners=4
        c1 = np.array([-base_length/2, 0, -base_width/2], dtype=float)
        c2 = np.array([ base_length/2, 0, -base_width/2], dtype=float)
        c3 = np.array([ base_length/2, 0, base_width/2], dtype=float)
        c4 = np.array([-base_length/2, 0, base_width/2], dtype=float)
        corners = [c1, c2, c3, c4]
    else:
        # Generic polygon in XZ plane
        for i in range(num_corners):
            theta = 2.0*math.pi*(i/num_corners)
            x = (base_length/2)*math.cos(theta)
            z = (base_width/2) *math.sin(theta)
            corners.append(np.array([x, 0, z], dtype=float))

    apex = np.array([0.0, apex_height, 0.0], dtype=float)

    def add_edge(a, b):
        if len(path)==0:
            path.append(a)
        path.append(b)

    # (1) Base loop
    for i in range(num_corners):
        A = corners[i]
        B = corners[(i+1)%num_corners]
        add_edge(A, B)

    # (2) Sides => c1->apex->c2->apex-> ... -> cN->apex->c1
    add_edge(corners[0], apex)
    for i in range(1, num_corners):
        add_edge(apex, corners[i])
        add_edge(corners[i], apex)
    add_edge(apex, corners[0])

    return path

###################################
##PHYSICS
###############################################################################

class PyramidPhysics:
    """
    Manages gravity, wave motion (x,y,z), bounce, rotation & thrust for a single pyramid.
    """
    def __init__(self):
        # Position & velocity
        self.position = np.array([0.0, 0.0, 0.0], dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=float)

        # Rotation => pitch, yaw, roll
        self.rotation = np.array([0.0, 0.0, 0.0], dtype=float)
        self.angular_velocity = np.array([0.0, 0.0, 0.0], dtype=float)

        # physics constants
        self.gravity = 9.8
        self.bounce_factor = 0.5
        self.mass = 1.0

        # wave config
        self.wave_axis_enable = {"x": False, "y": False, "z": False}
        self.wave_amplitude = {"x": 0.0, "y": 0.5, "z": 0.0}
        self.wave_frequency = {"x": 1.0, "y": 1.0, "z": 1.0}
        self.wave_phase = {"x": 0.0, "y": 0.0, "z": 0.0}

    def update(self, dt, current_time):
        # 1) gravity
        self.velocity[1] -= self.gravity * dt

        # 2) integrate linear
        self.position += self.velocity * dt

        # 3) integrate angular
        self.rotation += self.angular_velocity * dt

        # 4) bounce
        if self.position[1]<0:
            self.position[1] =0
            self.velocity[1] =-self.velocity[1]*self.bounce_factor

        # 5) wave offset
        wave_offset = np.array([0.0,0.0,0.0],dtype=float)
        for ax, idx in zip(["x","y","z"], [0,1,2]):
            if self.wave_axis_enable[ax]:
                amp = self.wave_amplitude[ax]
                freq = self.wave_frequency[ax]
                ph = self.wave_phase[ax]
                val = amp* math.sin(freq*current_time + ph)
                wave_offset[idx] = val
        self.position += wave_offset

    def apply_forward_thrust(self, force, dt):
        """
        Moves pyramid forward in local +Z direction (approx by yaw).
        """
        yaw_rad = math.radians(self.rotation[1])
        dir_vec = np.array([math.sin(yaw_rad), 0.0, math.cos(yaw_rad)], dtype=float)
        accel = (dir_vec*force)/ self.mass
        self.velocity+= accel*dt

###############################################################################
# PYRAMID
###############################################################################

class Pyramid:
    """
    Parametric pyramid with dynamic labeling, wave-based physics, corner offsets.
    Allows re-generating labels by changing pyramid_id (set_pyramid_id).
    """
    def __init__(self,
                 pyramid_id =1,
                 num_corners =4,
                 base_length =1.0,
                 base_width =1.0,
                 apex_height =2.0):

        self.pyramid_id = pyramid_id
        self.num_corners = num_corners
        self.base_length = base_length
        self.base_width = base_width
        self.apex_height = apex_height

        # generate labeling
        self.labels = generate_pyramid_labels(self.pyramid_id, num_corners=self.num_corners)

        # local path => c1->c2->...->cN->c1 + sides
        self.local_path = build_local_pyramid_path(
            num_corners = num_corners,
            base_length = base_length,
            base_width = base_width,
            apex_height = apex_height
        )

        # corner offsets
        self.corner_offsets = [np.array([0.0,0.0,0.0],dtype=float) for _ in range(num_corners)]

        # physics
        self.physics = PyramidPhysics()

    def set_pyramid_id(self, new_id):
        """
        Dynamically re-assign this pyramid's ID => re-generate labeling to remain unique.
        This is used if the master controller wants to replicate and rename it.
        """
        self.pyramid_id = new_id
        self.labels = generate_pyramid_labels(self.pyramid_id, self.num_corners)

    def update(self, dt, current_time):
        self.physics.update(dt, current_time)

    def get_transformed_path(self):
        """
        Returns final (x,y,z) points after corner offsets & physics transform.
        """
        mod_path = self._apply_corner_offsets_to_local_path(self.local_path)
        return self.transform_path(mod_path,
                                   self.physics.position,
                                   self.physics.rotation)

    def set_corner_offset(self, corner_index, offset_vec):
        """
        Locally warp corner corner_index by offset_vec.
        """
        if 0<= corner_index< self.num_corners:
            self.corner_offsets[corner_index] = np.array(offset_vec,dtype=float)

    def _apply_corner_offsets_to_local_path(self, path):
        corner_map = {}
        local_corners = path[: self.num_corners] # first num_corners => base corners
        for i in range(self.num_corners):
            old_pt = local_corners[i]
            c_off = self.corner_offsets[i]
            c_new = old_pt + c_off
            corner_map[old_pt.tobytes()] = c_new

        mod_path=[]
        for pt in path:
            key = pt.tobytes()
            if key in corner_map:
                mod_path.append(corner_map[key])
            else:
                mod_path.append(pt)
        return mod_path

    @staticmethod
    def transform_path(local_points, position, rotation):
        pitch = math.radians(rotation[0])
        yaw = math.radians(rotation[1])
        roll = math.radians(rotation[2])

        # rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(pitch), -math.sin(pitch)],
            [0, math.sin(pitch), math.cos(pitch)]
        ], dtype=float)

        Ry = np.array([
            [ math.cos(yaw), 0, math.sin(yaw)],
            [0, 1, 0],
            [-math.sin(yaw),0, math.cos(yaw)]
        ], dtype=float)

        Rz = np.array([
            [ math.cos(roll), -math.sin(roll), 0],
            [ math.sin(roll), math.cos(roll), 0],
            [0, 0, 1]
        ], dtype=float)

        R = Rz@ Ry@ Rx

        out_pts=[]
        for pt in local_points:
            rp = R @ pt
            wp = rp+ position
            out_pts.append(wp)
        return out_pts

###############################################################################
# DEMO 1: 5-SECOND CONSOLE
###############################################################################
def console_demo():
    print("\n** 5-second console wave test for a single pyramid **")
    from time import sleep

    p = Pyramid(pyramid_id=7, num_corners=4,
                base_length=2.0, base_width=1.5, apex_height=3.0)
    p.set_corner_offset(0,(0.0, 0.3, 0.0))

    p.physics.wave_axis_enable["x"] = True
    p.physics.wave_axis_enable["y"] = True
    p.physics.wave_amplitude["x"] = 0.2
    p.physics.wave_amplitude["y"] = 1.0
    p.physics.wave_frequency["x"] = 2.0
    p.physics.wave_frequency["y"] = 1.0

    start_t = time.time()
    last_t = start_t
    while True:
        now = time.time()
        elapsed= now- start_t
        if elapsed>5.0:
            break
        dt= now- last_t
        last_t= now

        p.update(dt, elapsed)
        pos= p.physics.position
        print(f"t={elapsed:.2f}s => pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})")
        sleep(0.1)

    print("\nDone. Final pos:", p.physics.position)

###############################################################################
# DEMO 2: PYGLET+OPENGL GUI
###############################################################################
def gui_demo():
    window= pyglet.window.Window(800,600,"Pyramid Demo")
    window.set_minimum_size(400,300)

    BG_BLACK = True
    start_t = time.time()
    last_t = start_t

    # create a pyramid
    pyramid = Pyramid(pyramid_id=7,num_corners=4,
                      base_length=2.0, base_width=1.5, apex_height=3.0)
    # corner offset
    pyramid.set_corner_offset(0,(0.0,0.3,0.0))
    # wave on X,Y
    pyramid.physics.wave_axis_enable["x"] = True
    pyramid.physics.wave_axis_enable["y"] = True
    pyramid.physics.wave_amplitude["x"] = 0.3
    pyramid.physics.wave_amplitude["y"] = 1.0
    pyramid.physics.wave_frequency["x"] = 2.0
    pyramid.physics.wave_frequency["y"] = 1.0
    # spin
    pyramid.physics.angular_velocity = np.array([0.0,10.0,0.0],dtype=float)

    dt=0.0

    def update(_dt):
        nonlocal dt,last_t
        now= time.time()
        dt= now- last_t
        last_t= now
        current_time= now- start_t

        pyramid.update(dt, current_time)

    pyglet.clock.schedule_interval(update, 1/60.0)

    @window.event
    def on_draw():
        nonlocal BG_BLACK
        if BG_BLACK:
            glClearColor(0,0,0,1)
        else:
            glClearColor(1,1,1,1)

        window.clear()
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL.PROJECTION)
        glLoadIdentity()
        aspect= float(window.width)/ window.height
        gluPerspective(60.0, aspect, 0.1,100.0)

        glMatrixMode(GL.MODELVIEW)
        glLoadIdentity()
        gluLookAt(0, 3, 12, 0,0,0, 0,1,0)

        now= time.time()- start_t
        glRotatef(now*15.0,0,1,0)

        draw_pyramid_wireframe(pyramid)

    @window.event
    def on_key_press(symbol,mods):
        nonlocal BG_BLACK
        if symbol== key.ESCAPE:
            window.close()
        elif symbol== key.SPACE:
            # reset pos & vel
            pyramid.physics.position[:]= 0
            pyramid.physics.velocity[:]= 0
        elif symbol== key.B:
            BG_BLACK= not BG_BLACK

    def draw_pyramid_wireframe(pyr):
        path_points= pyr.get_transformed_path()
        if len(path_points)<2:
            return

        glLineWidth(2.0)
        glColor3f(1,1,1)
        glBegin(GL_LINE_STRIP)
        for pt in path_points:
            glVertex3f(pt[0],pt[1],pt[2])
        glEnd()

    pyglet.app.run()

###############################################################################
def main():
    """
    By default => runs gui_demo. If you want the 5s console test, switch to console_demo().
    """
    #console_demo()
    gui_demo()

if __name__=="__main__":
    main()
