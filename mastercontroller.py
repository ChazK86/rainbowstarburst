#!/usr/bin/env python3
"""
master_controller.py

Significantly Enhanced for a "Globe" shape arrangement of pyramids:
1) Uses a subdivided icosahedron to generate triangular faces on a sphere.
2) Each face becomes the "base" of a triangular pyramid, apex outward,
   forming a cohesive "globe" or "shell" of pyramids around an invisible sphere.
3) Minimizes clipping vs. random distributions or simple "spike sphere."
4) Maintains existing features:
   - Edge-to-edge, Spike sphere, Grid, Star
   - Collisions & Particle System
   - Wave / Spin animation
   - Single-stroke bridging
   - Replication & Renaming w/ dynamic IDs
   - File export for each pyramid => "Pyramids/pyramid_{ID}.txt"

Requires:
  - Updated 'pyramid.py' with set_pyramid_id(...) and flexible local_path usage
"""

import os
import math
import random
from collections import deque

import numpy as np

# Import the advanced Pyramid class
from pyramid import Pyramid

# Particle Modes
PARTICLE_OFF    = "OFF"
PARTICLE_LOW    = "LOW"
PARTICLE_MEDIUM = "MEDIUM"
PARTICLE_HEAVY  = "HEAVY"
MAX_PARTICLES   = 2000
MAX_GLOBE_SUBDIVISIONS = 3

class Particle:
    """
    Basic collision-burst particle
    """
    def __init__(self, position, velocity, color, life=2.0):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.color    = color
        self.life     = life

    def update(self, dt, gravity=9.8):
        self.velocity[1] -= gravity*dt
        self.position += self.velocity*dt
        self.life -= dt

    def is_dead(self):
        return self.life <=0

# Basic animation modes
ANIMATION_WAVE_Y = "wave_y"
ANIMATION_SPIN   = "spin"
ANIMATION_PULSE  = "pulse_apex"

###############################################################################
# Icosahedron Generation & Subdivision
###############################################################################
# We'll define a function that returns a list of triangular faces for a
# subdivided icosahedron (unit sphere). Each face is a triple of 3D coords.

def generate_icosahedron_faces(subdivisions=0):
    """
    Returns a list of faces, where each face is (v1, v2, v3), each v is a np.array([x,y,z])
    on the unit sphere. If subdivisions>0, we subdiv each triangular face, normalizing
    to keep them on sphere, to approximate a geodesic sphere.

    The result is a list of triangular faces that can be used to place triangular pyramids
    around an invisible core.
    """
    try:
        subdivisions = int(subdivisions)
    except (TypeError, ValueError):
        subdivisions = 0
    subdivisions = max(0, min(MAX_GLOBE_SUBDIVISIONS, subdivisions))

    # Basic icosahedron coordinates (golden ratio approach)
    t  = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [
        np.array([-1,  t,  0],dtype=float),
        np.array([ 1,  t,  0],dtype=float),
        np.array([-1, -t,  0],dtype=float),
        np.array([ 1, -t,  0],dtype=float),

        np.array([ 0, -1,  t],dtype=float),
        np.array([ 0,  1,  t],dtype=float),
        np.array([ 0, -1, -t],dtype=float),
        np.array([ 0,  1, -t],dtype=float),

        np.array([ t,  0, -1],dtype=float),
        np.array([ t,  0,  1],dtype=float),
        np.array([-t,  0, -1],dtype=float),
        np.array([-t,  0,  1],dtype=float)
    ]

    # normalize them to unit
    for i in range(len(verts)):
        verts[i] /= np.linalg.norm(verts[i])

    # The 20 triangular faces of an icosahedron, referencing above vertices
    faces_idx = [
        ( 0, 11,  5), ( 0,  5,  1), ( 0,  1,  7), ( 0,  7, 10), ( 0, 10, 11),
        ( 1,  5,  9), ( 5, 11,  4), (11, 10,  2), (10,  7,  6), ( 7,  1,  8),
        ( 3,  9,  4), ( 3,  4,  2), ( 3,  2,  6), ( 3,  6,  8), ( 3,  8,  9),
        ( 4,  9,  5), ( 2,  4, 11), ( 6,  2, 10), ( 8,  6,  7), ( 9,  8,  1)
    ]

    faces = []
    for (a,b,c) in faces_idx:
        faces.append((verts[a], verts[b], verts[c]))

    if subdivisions<=0:
        return faces

    # SUBDIVIDE each face => 4 smaller faces => recursively if needed
    for _ in range(subdivisions):
        new_faces = []
        for f in faces:
            v1,v2,v3 = f
            # midpoints
            m12 = normalize_vec(0.5*(v1+v2))
            m23 = normalize_vec(0.5*(v2+v3))
            m31 = normalize_vec(0.5*(v3+v1))

            # 4 new faces
            new_faces.append((v1, m12, m31))
            new_faces.append((v2, m23, m12))
            new_faces.append((v3, m31, m23))
            new_faces.append((m12,m23,m31))
        faces = new_faces

    return faces

def normalize_vec(v):
    """
    normalizes a 3D vector to unit length
    """
    d = np.linalg.norm(v)
    if d<1e-9:
        return v
    return v/d


def rotation_aligning_apex(direction):
    """Return Euler angles that rotate local +Y toward ``direction``.

    ``Pyramid.transform_path`` applies ``Rz @ Ry @ Rx``.  Holding yaw at zero
    gives a stable two-angle solution and avoids the half-sphere collapse caused
    by leaving every spike upright.
    """
    direction = normalize_vec(np.asarray(direction, dtype=float))
    if np.linalg.norm(direction) < 1e-9:
        return np.zeros(3, dtype=float)
    x, y, z = direction
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, z))))
    roll = math.degrees(math.atan2(-x, y))
    return np.array([pitch, 0.0, roll], dtype=float)

###############################################################################
# MASTER CONTROLLER
###############################################################################

class MasterController:
    """
    Incorporates all prior functionality, plus a new 'init_globe_icosahedron' arrangement
    that creates triangular-based pyramids around a subdivided icosahedron for
    a 'globe' shape with minimal overlap and apex outward from the core.
    """
    def __init__(self, export_directory="Pyramids"):
        self.pyramids = []
        self.particles= []
        self.current_particle_mode = PARTICLE_OFF

        self.user_sphere_pos    = np.array([0,0,0],dtype=float)
        self.user_sphere_radius = 0.5

        # Keep generated definitions in one resolved directory.  The exporter
        # only manages files matching ``pyramid_<integer>.txt`` inside it.
        self.export_directory = os.path.abspath(export_directory)
        os.makedirs(self.export_directory, exist_ok=True)

        # export management configuration
        self.export_enabled = True
        self.export_async = False
        self.exports_per_update = 10
        self._export_queue = deque()

    # ------------------------------------------------------------------------
    # ARRANGEMENTS
    # ------------------------------------------------------------------------
    def init_globe_icosahedron(self,
                               subdivisions = 1,
                               apex_offset  = 0.2,
                               base_scale   = 1.0,
                               start_id     = 1):
        """
        Creates a 'globe' shape by placing triangular pyramids on each face of a
        subdivided icosahedron. The 'apex_offset' is how far outward from the
        face (in the face normal direction) the apex is placed. 'base_scale'
        scales the base corners outward or inward for spacing adjustments.

        - subdivisions=0 => basic icosahedron => 20 pyramids
        - subdivisions=1 => 80 pyramids
        - subdivisions=2 => 320 pyramids, etc.

        The new pyramids use 'num_corners=3' forcibly, since the base is triangular.
        We override their local_path so the base corners are exactly those of the face.
        We set the apex outward from the face center by 'apex_offset'.

        NOTE:
        - We do not automatically wave or spin these pyramids unless you set an
          animation mode or do it manually.
        - We store each pyramid => 'Pyramids/pyramid_{ID}.txt' with unique ID.
        """
        self.clear_pyramids(prune_exports=False)

        faces = generate_icosahedron_faces(subdivisions=subdivisions)
        pid = start_id

        for tri in faces:
            # tri => (v1,v2,v3), each on unit sphere
            # We'll define a new triangular pyramid
            # 1) scale corners outward or inward => base corners
            c1 = tri[0]* base_scale
            c2 = tri[1]* base_scale
            c3 = tri[2]* base_scale

            # face normal => average => outward direction
            face_center = (c1 + c2 + c3)/3.0
            face_normal = normalize_vec(face_center)
            # apex => face_center + (face_normal* apex_offset)
            apex = face_center + face_normal* apex_offset

            # We'll build a new pyramid with pyramid_id=pid, corners=3
            p = Pyramid(
                pyramid_id   = pid,
                num_corners  = 3,   # triangular base
                base_length  = 1.0, # placeholders
                base_width   = 1.0, # placeholders
                apex_height  = 1.0  # placeholders
            )
            pid+=1

            # override local_path so that c1->c2->c3->c1, then sides => c1->apex->c2->apex->c3->apex->c1
            custom_path = []
            def add_edge(a, b):
                if len(custom_path)==0:
                    custom_path.append(a)
                custom_path.append(b)

            # base loop
            add_edge(c1, c2)
            add_edge(c2, c3)
            add_edge(c3, c1)

            # sides
            add_edge(c1, apex)
            add_edge(apex, c2)
            add_edge(c2, apex)
            add_edge(apex, c3)
            add_edge(c3, apex)
            add_edge(apex, c1)

            # store custom path
            p.local_path = custom_path
            p.physics.gravity = 0.0
            p.physics.ground_collision_enabled = False

            # We'll place the pyramid's physics.position= [0,0,0], no wave by default.
            # if you want wave/spin, do set_animation_mode(...) or individually set it.

            self.pyramids.append(p)
            self.export_pyramid_file(p)

        self.sync_export_files()

    def init_edge_to_edge_pyramids(self,
                                   count=5,
                                   wave_offset=0.3,
                                   base_length=1.0,
                                   base_width=1.0,
                                   apex_height=2.0):
        """
        The older method => line arrangement
        """
        self.clear_pyramids(prune_exports=False)

        x_offset= base_length
        start_x = -(count - 1) * x_offset / 2.0
        for i in range(count):
            pid= i+1
            p= Pyramid(
                pyramid_id  = pid,
                num_corners = 4,
                base_length = base_length,
                base_width  = base_width,
                apex_height = apex_height
            )
            p.physics.position[0]= start_x + i*x_offset
            p.physics.gravity=0.0
            p.physics.ground_collision_enabled=False
            # wave
            p.physics.wave_axis_enable["y"]=True
            p.physics.wave_phase["y"]= i*wave_offset
            p.physics.wave_amplitude["y"]=0.5
            self.pyramids.append(p)
            self.export_pyramid_file(p)

        self.sync_export_files()

    def init_spike_sphere_pyramids(self,
                                   count=8,
                                   sphere_radius=5.0,
                                   base_length=1.0,
                                   base_width=1.0,
                                   apex_height=2.0):
        """
        The older spike approach => random fibonacci faces
        """
        self.clear_pyramids(prune_exports=False)
        if count<1:
            self.sync_export_files()
            return

        phi= math.pi*(3.0- math.sqrt(5.0))
        for i in range(count):
            frac= i/float(count-1) if count>1 else 0.0
            y= 1.0-(frac*2.0)
            ry= math.sqrt(max(0,1-y*y))
            theta= phi*i
            x= math.cos(theta)* ry
            z= math.sin(theta)* ry

            dir_vec= np.array([x,y,z],dtype=float)
            norm= np.linalg.norm(dir_vec)
            if norm>1e-9: dir_vec/= norm

            pid= i+1
            p= Pyramid(
                pyramid_id   = pid,
                num_corners  =4,
                base_length  = base_length,
                base_width   = base_width,
                apex_height  = apex_height
            )
            p.physics.position= dir_vec*sphere_radius
            p.physics.rotation= rotation_aligning_apex(dir_vec)
            p.physics.gravity=0.0
            p.physics.ground_collision_enabled=False
            # apex outward => wave off
            p.physics.wave_axis_enable["x"]=False
            p.physics.wave_axis_enable["y"]=False
            p.physics.wave_axis_enable["z"]=False

            self.pyramids.append(p)
            self.export_pyramid_file(p)

        self.sync_export_files()

    def init_grid_pyramids(self,
                           rows=3,
                           cols=3,
                           spacing_x=2.0,
                           spacing_z=2.0,
                           base_length=1.0,
                           base_width=1.0,
                           apex_height=2.0):
        """
        The older 2D grid approach
        """
        self.clear_pyramids(prune_exports=False)
        pid=1
        start_x= -(cols-1)*spacing_x/2
        start_z= -(rows-1)*spacing_z/2
        for r in range(rows):
            for c in range(cols):
                p= Pyramid(
                    pyramid_id= pid,
                    num_corners=4,
                    base_length= base_length,
                    base_width= base_width,
                    apex_height= apex_height
                )
                pid+=1
                px= start_x + c* spacing_x
                pz= start_z + r* spacing_z
                p.physics.position= np.array([px,0,pz],dtype=float)
                p.physics.gravity=0.0
                p.physics.ground_collision_enabled=False
                # wave on y
                p.physics.wave_axis_enable["y"]=True
                p.physics.wave_phase["y"]= (r+c)*0.3
                p.physics.wave_amplitude["y"]=0.4

                self.pyramids.append(p)
                self.export_pyramid_file(p)

        self.sync_export_files()

    def init_star_formation(self,
                            subdivisions=1,
                            core_radius=3.0,
                            spike_height=2.4,
                            start_id=1):
        """Build a closed, full star from edge-sharing radial pyramids.

        The bases are the triangular faces of one subdivided icosahedral core.
        Adjacent pyramids therefore share the exact same base edge instead of
        floating at unrelated Fibonacci-sphere positions.  Every apex extends
        outward from its face, producing a joined SpikeSphere / full star.

        ``subdivisions=0`` creates 20 broad spikes and ``subdivisions=1`` creates
        80 finer spikes.  The global safety limit also applies here.
        """
        self.clear_pyramids(prune_exports=False)

        faces = generate_icosahedron_faces(subdivisions=subdivisions)
        pid = int(start_id)
        for v1, v2, v3 in faces:
            c1 = np.asarray(v1, dtype=float) * core_radius
            c2 = np.asarray(v2, dtype=float) * core_radius
            c3 = np.asarray(v3, dtype=float) * core_radius
            face_center = (c1 + c2 + c3) / 3.0
            face_normal = normalize_vec(face_center)
            apex = face_center + face_normal * spike_height

            p = Pyramid(
                pyramid_id=pid,
                num_corners=3,
                base_length=core_radius,
                base_width=core_radius,
                apex_height=spike_height,
            )
            pid += 1

            # A continuous path around the shared triangular base and out along
            # every spike edge.  The base vertices are reused verbatim between
            # neighboring faces, which is what guarantees edge-to-edge contact.
            p.local_path = [
                c1, c2, c3, c1,
                apex, c2, apex, c3, apex, c1,
            ]
            p.physics.gravity = 0.0
            p.physics.ground_collision_enabled = False

            self.pyramids.append(p)
            self.export_pyramid_file(p)

        self.sync_export_files()

    # ------------------------------------------------------------------------
    # MULTI-PYRAMID MANAGEMENT
    # ------------------------------------------------------------------------
    def clear_pyramids(self, prune_exports=True):
        self.pyramids=[]
        self.particles=[]
        if self.export_async:
            self._export_queue.clear()
        if prune_exports:
            self.sync_export_files()

    def add_pyramid(self, **kwargs):
        """
        Create new pyramid => file export
        """
        p= Pyramid(**kwargs)
        self.pyramids.append(p)
        self.export_pyramid_file(p)
        return p

    def rename_pyramid(self, old_id, new_id):
        """
        Re-ID a pyramid => set_pyramid_id => new labels => export
        """
        targ=None
        for pm in self.pyramids:
            if pm.pyramid_id== old_id:
                targ= pm
                break
        if targ is None:
            print(f"[rename_pyramid] => no pyramid with ID={old_id}")
            return
        targ.set_pyramid_id(new_id)
        self.export_pyramid_file(targ)
        self.sync_export_files()

    def replicate_pyramid(self, source_id, new_id):
        src=None
        for pm in self.pyramids:
            if pm.pyramid_id== source_id:
                src= pm
                break
        if src is None:
            print(f"[replicate_pyramid] => no pyramid with ID={source_id}")
            return None

        clone= Pyramid(
            pyramid_id   = new_id,
            num_corners  = src.num_corners,
            base_length  = src.base_length,
            base_width   = src.base_width,
            apex_height  = src.apex_height
        )
        # copy corner offsets
        for i in range(len(clone.corner_offsets)):
            clone.corner_offsets[i]= np.copy(src.corner_offsets[i])
        # copy physics
        clone.physics.position         = np.copy(src.physics.position)
        clone.physics.velocity         = np.copy(src.physics.velocity)
        clone.physics.rotation         = np.copy(src.physics.rotation)
        clone.physics.angular_velocity = np.copy(src.physics.angular_velocity)
        clone.physics.wave_axis_enable = dict(src.physics.wave_axis_enable)
        clone.physics.wave_amplitude   = dict(src.physics.wave_amplitude)
        clone.physics.wave_frequency   = dict(src.physics.wave_frequency)
        clone.physics.wave_phase       = dict(src.physics.wave_phase)
        clone.physics.gravity          = src.physics.gravity
        clone.physics.bounce_factor    = src.physics.bounce_factor
        clone.physics.mass             = src.physics.mass
        clone.physics.ground_collision_enabled = src.physics.ground_collision_enabled

        # if custom local_path was set => also replicate that
        # e.g. for triangular face pyramids
        clone.local_path= [pt.copy() for pt in src.local_path]

        self.pyramids.append(clone)
        self.export_pyramid_file(clone)
        return clone

    def remove_pyramid(self, pyramid_id):
        self.pyramids= [p for p in self.pyramids if p.pyramid_id != pyramid_id]
        self.sync_export_files()

    # ------------------------------------------------------------------------
    # EXPORT CONFIGURATION
    # ------------------------------------------------------------------------
    def configure_export(self, *, enabled=None, async_mode=None, exports_per_update=None):
        """Configure how pyramid exports are written to disk.

        Args:
            enabled: Toggle exporting on or off.
            async_mode: When True, export requests are queued and flushed during
                :meth:`update` to avoid blocking the render loop.
            exports_per_update: Maximum number of queued exports to process on
                each update tick when ``async_mode`` is enabled.
        """

        if enabled is not None:
            self.export_enabled = bool(enabled)
            if not self.export_enabled:
                self._export_queue.clear()

        if async_mode is not None:
            self.export_async = bool(async_mode)
            if not self.export_async:
                self._export_queue.clear()

        if exports_per_update is not None:
            try:
                exports_per_update = int(exports_per_update)
            except (TypeError, ValueError):
                exports_per_update = self.exports_per_update
            self.exports_per_update = max(1, exports_per_update)

    def export_pyramid_file(self, pyramid):
        """Request an export for ``pyramid`` respecting the configured policy."""

        if not self.export_enabled:
            return

        if self.export_async:
            self._export_queue.append(pyramid)
        else:
            self._write_pyramid_file(pyramid)

    def flush_export_queue(self):
        """Process queued exports if asynchronous exporting is enabled."""

        if not (self.export_enabled and self.export_async):
            return

        processed = 0
        while self._export_queue and processed < self.exports_per_update:
            pyramid = self._export_queue.popleft()
            self._write_pyramid_file(pyramid)
            processed += 1

    def sync_export_files(self):
        """Remove stale managed exports so the directory mirrors the live scene.

        Only exact ``pyramid_<integer>.txt`` files in ``export_directory`` are
        touched.  Other files and subdirectories are deliberately ignored.
        """
        if not self.export_enabled:
            return

        active_ids = {int(p.pyramid_id) for p in self.pyramids}
        try:
            entries = os.scandir(self.export_directory)
        except FileNotFoundError:
            os.makedirs(self.export_directory, exist_ok=True)
            return

        with entries:
            for entry in entries:
                if not entry.is_file(follow_symlinks=False):
                    continue
                name = entry.name
                if not (name.startswith("pyramid_") and name.endswith(".txt")):
                    continue
                id_text = name[len("pyramid_"):-len(".txt")]
                if not id_text.isdigit():
                    continue
                if int(id_text) not in active_ids:
                    os.remove(entry.path)

    # ------------------------------------------------------------------------
    # ANIMATION
    # ------------------------------------------------------------------------
    def set_animation_mode(self, mode):
        # Animation modes are mutually exclusive.  The old implementation left
        # angular velocity running when switching from SPIN to WAVE/PULSE.
        for p in self.pyramids:
            p.physics.wave_axis_enable["x"]=False
            p.physics.wave_axis_enable["y"]=False
            p.physics.wave_axis_enable["z"]=False
            p.physics.wave_offset[:]=0.0
            p.physics.angular_velocity[:]=0.0

        if mode== ANIMATION_WAVE_Y:
            for p in self.pyramids:
                p.physics.wave_axis_enable["y"]=True
                p.physics.wave_amplitude["y"]=1.0
                p.physics.wave_frequency["y"]=1.0
        elif mode== ANIMATION_SPIN:
            for p in self.pyramids:
                p.physics.angular_velocity= np.array([0.0, 30.0, 0.0],dtype=float)
        elif mode== ANIMATION_PULSE:
            for p in self.pyramids:
                p.physics.wave_axis_enable["y"]=True
                p.physics.wave_amplitude["y"]=2.0
                p.physics.wave_frequency["y"]=2.0

    # ------------------------------------------------------------------------
    # UPDATE
    # ------------------------------------------------------------------------
    def update(self, dt, current_time, user_cursor_pos=None):
        for p in self.pyramids:
            p.update(dt, current_time)

        if user_cursor_pos is not None:
            self.user_sphere_pos= np.array(user_cursor_pos, dtype=float)
            self.check_collisions()

        # update particles
        for par in self.particles:
            par.update(dt)
        # prune
        self.particles= [pp for pp in self.particles if not pp.is_dead()]
        if len(self.particles)> MAX_PARTICLES:
            self.particles= self.particles[-MAX_PARTICLES:]

        # write any deferred exports without blocking the render/update loop
        self.flush_export_queue()

    def check_collisions(self):
        """Detect collisions using an oriented bounding box derived from each path."""

        user_center = self.user_sphere_pos
        radius = self.user_sphere_radius
        radius_sq = radius * radius

        for pyramid in self.pyramids:
            path = pyramid.get_transformed_path()
            if not path:
                continue

            pts = np.asarray(path, dtype=float)
            if pts.shape[0] < 3:
                # Degenerate path, fall back to a conservative sphere around position.
                px, py, pz = pyramid.physics.position
                approx_center = np.array([px, py + (pyramid.apex_height * 0.5), pz], dtype=float)
                approx_radius = max(pyramid.base_length, pyramid.base_width) * 0.5 + pyramid.apex_height * 0.5
                if np.linalg.norm(approx_center - user_center) < (approx_radius + radius):
                    self.handle_collision(approx_center)
                continue

            center = pts.mean(axis=0)
            centered = pts - center

            if np.allclose(centered, 0.0):
                axes = np.eye(3)
            else:
                cov = centered.T @ centered
                try:
                    eigenvalues, eigenvectors = np.linalg.eigh(cov)
                except np.linalg.LinAlgError:
                    axes = np.eye(3)
                else:
                    order = np.argsort(eigenvalues)[::-1]
                    axes = eigenvectors[:, order]
                    for i in range(axes.shape[1]):
                        norm = np.linalg.norm(axes[:, i])
                        if norm > 1e-8:
                            axes[:, i] /= norm
                        else:
                            axes[:, i] = np.eye(3)[:, i]

            local_coords = centered @ axes
            extents = np.max(np.abs(local_coords), axis=0)
            # Apply a small safety margin so narrow shapes still register collisions.
            margin = max(pyramid.base_length, pyramid.base_width, pyramid.apex_height) * 0.05
            extents = np.maximum(extents, margin)

            rel = user_center - center
            rel_local = rel @ axes
            closest_local = np.clip(rel_local, -extents, extents)
            closest_point = center + axes @ closest_local

            dist_sq = np.sum((closest_point - user_center) ** 2)
            if dist_sq <= radius_sq:
                self.handle_collision(closest_point)

    def handle_collision(self, origin):
        if self.current_particle_mode==PARTICLE_OFF:
            return
        elif self.current_particle_mode==PARTICLE_LOW:
            self.spawn_particles(50, origin)
        elif self.current_particle_mode==PARTICLE_MEDIUM:
            self.spawn_particles(200, origin)
        elif self.current_particle_mode==PARTICLE_HEAVY:
            self.spawn_particles(500, origin)

    def spawn_particles(self, num, origin):
        for _ in range(num):
            vx= (random.random()-0.5)*8
            vy= random.random()*8
            vz= (random.random()-0.5)*8
            c= (random.random(), random.random(), random.random())
            life=1.0+ random.random()*2.0
            self.particles.append(Particle(origin,[vx,vy,vz], c, life))

    def set_particle_mode(self, mode):
        self.current_particle_mode= mode

    # ------------------------------------------------------------------------
    # SINGLE-STROKE BRIDGING
    # ------------------------------------------------------------------------
    def build_global_path(self, close_loop=False, star_bridge=False, star_step=None):
        """
        merges each pyramid's path => single line. bridging => last->first
        If star_bridge= True => reorder with star index (step= n//2 or user-defined)
        """
        if star_bridge and len(self.pyramids)>2:
            n= len(self.pyramids)
            if star_step is None:
                star_step= n//2
            used= [False]*n
            index_list=[]
            i=0
            while True:
                index_list.append(i)
                used[i]=True
                i=(i+ star_step)% n
                if used[i]:
                    break
            reorder= [self.pyramids[j] for j in index_list]
        else:
            reorder= self.pyramids

        all_points=[]
        for i,p in enumerate(reorder):
            tpath= p.get_transformed_path()
            if not all_points:
                all_points.extend(tpath)
            else:
                all_points.append(tpath[0])
                all_points.extend(tpath[1:])

        if close_loop and len(all_points)>1:
            all_points.append(all_points[0])

        if len(all_points)<2:
            return (all_points,1.0,[0.0])

        dists=[0.0]
        total_len=0.0
        for i in range(1,len(all_points)):
            seg_len= np.linalg.norm(all_points[i]- all_points[i-1])
            total_len+= seg_len
            dists.append(total_len)
        return (all_points,total_len,dists)

    # ------------------------------------------------------------------------
    # EXPORT
    # ------------------------------------------------------------------------
    def _write_pyramid_file(self, p):
        """
        Writes param & labeling => "Pyramids/pyramid_{p.pyramid_id}.txt"
        capturing shape & physics state.
        """
        fname= f"pyramid_{p.pyramid_id}.txt"
        path= os.path.join(self.export_directory,fname)

        with open(path,"w") as f:
            f.write(f"** Pyramid ID= {p.pyramid_id} **\n")
            f.write(f"num_corners= {p.num_corners}\n")
            f.write(f"base= {p.base_length} x {p.base_width}\n")
            f.write(f"apex_height= {p.apex_height}\n\n")

            f.write("LABELS:\n")
            for k,v in p.labels.items():
                f.write(f"  {k} => {v}\n")

            f.write("\nCORNER OFFSETS:\n")
            for i, c_off in enumerate(p.corner_offsets):
                f.write(f"  corner{i}: {c_off}\n")

            f.write("\nPHYSICS:\n")
            f.write(f"  position= {p.physics.position}\n")
            f.write(f"  velocity= {p.physics.velocity}\n")
            f.write(f"  rotation= {p.physics.rotation}\n")
            f.write(f"  angular_velocity= {p.physics.angular_velocity}\n")
            f.write(f"  wave_axis_enable= {p.physics.wave_axis_enable}\n")
            f.write(f"  wave_amplitude= {p.physics.wave_amplitude}\n")
            f.write(f"  wave_frequency= {p.physics.wave_frequency}\n")
            f.write(f"  wave_phase= {p.physics.wave_phase}\n")
            f.write(f"  gravity= {p.physics.gravity}, bounce= {p.physics.bounce_factor}, mass= {p.physics.mass}\n")
            f.write(f"  ground_collision_enabled= {p.physics.ground_collision_enabled}\n")

        print(f"[export_pyramid_file] => Created {path}")


###############################################################################
# TEST

###############################################################################
def main():
    """
    Quick test => create a 'globe' using subdivided icosahedron => 1 subdivision => 80 pyramids,
    each triangular-based. Then do a short update loop, build global path, done.
    """
    import time

    mc= MasterController()
    # build a subdivided icosahedron => ~80 faces => 80 triangular pyramids => a 'globe'
    mc.init_globe_icosahedron(subdivisions=1, apex_offset=0.3, base_scale=1.0, start_id=1)

    # partial update loop
    start= time.time()
    last= start
    while True:
        now= time.time()
        if (now- start)>3.0:
            break
        dt= now- last
        last= now

        # example user cursor => small circle
        x= 2.0* math.sin(now)
        z= 2.0* math.cos(now)
        mc.update(dt, now-start, user_cursor_pos=(x,0,z))
        time.sleep(0.2)

    # build path => bridging. (Since it's triangular, star bridging might not be beneficial,
    # but you can set star_bridge=True if you want a different path.)
    points, length, dists= mc.build_global_path(close_loop=False, star_bridge=False)
    print(f"\nFinal path => {len(points)} points, total_len= {length:.2f}")
    print(f"#pyramids= {len(mc.pyramids)}, #particles= {len(mc.particles)}\n")


if __name__=="__main__":
    main()
