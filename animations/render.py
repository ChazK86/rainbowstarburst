"""Pure transient geometry transform used by the OpenGL renderer and tests."""

from __future__ import annotations

import math

import numpy as np


def _rotation_matrix_xyz(degrees):
    pitch, yaw, roll = np.radians(degrees)
    cx, sx = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cz, sz = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def apply_render_state(path, topology, index, state):
    """Return a modulated copy while leaving every authored point untouched.

    Shared bases receive only one formation-wide scale, so the full Star cannot
    crack apart.  Tip scale, radial travel, and rotation affect repeated apex
    vertices only; base vertices remain exactly edge-sharing.
    """
    points = np.asarray(path, dtype=float).copy()
    if not len(points):
        return points
    center = topology.centers[index]
    apex = topology.apexes[index]
    normal = topology.normals[index]
    if topology.shared_surface:
        global_scale = float(np.mean(state.uniform_scale))
        points = topology.origin + (points - topology.origin) * global_scale
        center = topology.origin + (center - topology.origin) * global_scale
        apex = topology.origin + (apex - topology.origin) * global_scale
    else:
        points = center + (points - center) * float(state.uniform_scale[index])
        apex = center + (apex - center) * float(state.uniform_scale[index])
    apex_mask = np.linalg.norm(points - apex, axis=1) < 1e-5
    tip_vector = apex - center
    if np.any(state.rotation_offset[index]):
        tip_vector = _rotation_matrix_xyz(state.rotation_offset[index]) @ tip_vector
    new_apex = center + tip_vector * float(state.apex_scale[index]) + normal * float(state.radial_offset[index])
    points[apex_mask] = new_apex
    return points
