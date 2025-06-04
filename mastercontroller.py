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
import numpy as np

# Import the advanced Pyramid class
from pyramid import Pyramid

# Particle Modes
PARTICLE_OFF    = "OFF"
PARTICLE_LOW    = "LOW"
PARTICLE_MEDIUM = "MEDIUM"
PARTICLE_HEAVY  = "HEAVY"
MAX_PARTICLES   = 2000

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

###############################################################################
# MASTER CONTROLLER
###############################################################################

class MasterController:
    """
    Incorporates all prior functionality, plus a new 'init_globe_icosahedron' arrangement
    that creates triangular-based pyramids around a subdivided icosahedron for
    a 'globe' shape with minimal overlap and apex outward from the core.
    """
    def __init__(self):
        self.pyramids = []
        self.particles= []
        self.current_particle_mode = PARTICLE_OFF

        self.user_sphere_pos    = np.array([0,0,0],dtype=float)
        self.user_sphere_radius = 0.5

        # ensure subdir
        os.makedirs("Pyramids", exist_ok=True)

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
        self.clear_pyramids()

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

            # We'll place the pyramid's physics.position= [0,0,0], no wave by default.
            # if you want wave/spin, do set_animation_mode(...) or individually set it.

            self.pyramids.append(p)
            self.export_pyramid_file(p)

    def init_edge_to_edge_pyramids(self,
                                   count=5,
                                   wave_offset=0.3,
                                   base_length=1.0,
                                   base_width=1.0,
                                   apex_height=2.0):
        """
        The older method => line arrangement
        """
        self.clear_pyramids()

        x_offset= base_length
        for i in range(count):
            pid= i+1
            p= Pyramid(
                pyramid_id  = pid,
                num_corners = 4,
                base_length = base_length,
                base_width  = base_width,
                apex_height = apex_height
            )
            p.physics.position[0]= i*x_offset
            # wave
            p.physics.wave_axis_enable["y"]=True
            p.physics.wave_phase["y"]= i*wave_offset
            p.physics.wave_amplitude["y"]=0.5
            self.pyramids.append(p)
            self.export_pyramid_file(p)

    def init_spike_sphere_pyramids(self,
                                   count=8,
                                   sphere_radius=5.0,
                                   base_length=1.0,
                                   base_width=1.0,
                                   apex_height=2.0):
        """
        The older spike approach => random fibonacci faces
        """
        self.clear_pyramids()
        if count<1:return

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
            # apex outward => wave off
            p.physics.wave_axis_enable["x"]=False
            p.physics.wave_axis_enable["y"]=False
            p.physics.wave_axis_enable["z"]=False

            self.pyramids.append(p)
            self.export_pyramid_file(p)

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
        self.clear_pyramids()
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
                # wave on y
                p.physics.wave_axis_enable["y"]=True
                p.physics.wave_phase["y"]= (r+c)*0.3
                p.physics.wave_amplitude["y"]=0.4

                self.pyramids.append(p)
                self.export_pyramid_file(p)

    # ------------------------------------------------------------------------
    # MULTI-PYRAMID MANAGEMENT
    # ------------------------------------------------------------------------
    def clear_pyramids(self):
        self.pyramids=[]

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

        # if custom local_path was set => also replicate that
        # e.g. for triangular face pyramids
        clone.local_path= [pt.copy() for pt in src.local_path]

        self.pyramids.append(clone)
        self.export_pyramid_file(clone)
        return clone

    def remove_pyramid(self, pyramid_id):
        self.pyramids= [p for p in self.pyramids if p.pyramid_id != pyramid_id]

    # ------------------------------------------------------------------------
    # ANIMATION
    # ------------------------------------------------------------------------
    def set_animation_mode(self, mode):
        if mode== ANIMATION_WAVE_Y:
            for p in self.pyramids:
                p.physics.wave_axis_enable["x"]=False
                p.physics.wave_axis_enable["y"]=True
                p.physics.wave_axis_enable["z"]=False
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
        else:
            # disable wave/spin
            for p in self.pyramids:
                p.physics.wave_axis_enable["x"]=False
                p.physics.wave_axis_enable["y"]=False
                p.physics.wave_axis_enable["z"]=False
                p.physics.angular_velocity[:]=0.0

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

    def check_collisions(self):
        for p in self.pyramids:
            # bounding sphere approx
            px,py,pz= p.physics.position
            r_approx= max(p.base_length,p.base_width)/2 + p.apex_height*0.5
            # BUT if local_path was custom => e.g. triangular => we might do our own approach
            # We'll just do the same approach unless you want to parse the local_path bounding
            center= np.array([px, py+(p.apex_height*0.5), pz],dtype=float)
            dist= np.linalg.norm(center- self.user_sphere_pos)
            if dist < (r_approx+ self.user_sphere_radius):
                self.handle_collision(center)

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
    def export_pyramid_file(self, p):
        """
        Writes param & labeling => "Pyramids/pyramid_{p.pyramid_id}.txt"
        capturing shape & physics state.
        """
        subdir="Pyramids"
        fname= f"pyramid_{p.pyramid_id}.txt"
        path= os.path.join(subdir,fname)

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
