#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy

import numpy as np
import trimesh

from ..commandline_logging import get_logger
from ..io import representation

log = get_logger(__name__)


def get_vertex_id(x, vertices):
    return np.logical_and(
        np.logical_and(
            vertices[:, 0] == x[0],
            vertices[:, 1] == x[1]),
        vertices[:, 2] == x[2]
    ).nonzero()[0][0]


def create_box(mesh, oriented=True, scale=1.0):
    """Create a box element.
    """

    if oriented:
        half_ext = mesh.bounding_box_oriented.primitive.extents
    else:
        half_ext = mesh.bounding_box.primitive.extents

    half_ext = np.array(half_ext) * scale

    return representation.Box(size=half_ext)


def create_sphere(mesh, oriented=True, scale=1.0):
    """ Create a sphere """

    if oriented:
        half_ext = mesh.bounding_box_oriented.primitive.extents
    else:
        half_ext = mesh.bounding_box.primitive.extents

    r = np.amax(half_ext)

    return representation.Sphere(radius=r * 0.5 * scale)


def create_cylinder(mesh, oriented=True, scale=1.0):
    """Create a cylinder.
    """

    if oriented:
        half_ext = deepcopy(mesh.bounding_box_oriented.primitive.extents)
    else:
        half_ext = deepcopy(mesh.bounding_box.primitive.extents)

    # Find the length and the axis
    axis = 'x'
    length = 2.0 * half_ext[0] * scale
    for (i, ax) in enumerate(['y', 'z']):
        if half_ext[i + 1] * 2.0 > length:
            length = half_ext[i + 1]
            axis = ax

    # Get the second largest
    half_ext.sort()
    radius = half_ext[1] * 0.5 * scale

    return representation.Cylinder(radius=radius, length=length)


def get_reflection_matrix(point=np.array((0, 0, 0)), normal=np.array((0, 1, 0))):
    return trimesh.transformations.reflection_matrix(point, normal)


def improve_mesh(mesh):
    mesh.fix_normals()
    mesh.fill_holes()
    mesh.merge_vertices()
    mesh.remove_duplicate_faces()
    mesh.remove_infinite_values()
    mesh.remove_unreferenced_vertices()
    return mesh


def reduce_mesh(mesh, factor):
    mesh = improve_mesh(mesh)
    n = np.ceil(factor * len(mesh.faces))
    out = mesh.simplify_quadratic_decimation(n)
    v = len(mesh.vertices)
    v_ = len(out.vertices)
    f = len(mesh.faces)
    f_ = len(out.faces)
    log.info(f"Reduced {f} -> {f_} ({np.round(1000 * f_ / f) / 10}%) faces and {v} -> {v_} ({np.round(1000 * v_ / v) / 10}%) vertices")
    return out


def identical(mesh_a, mesh_b):
    if mesh_a == mesh_b:
        return True
    assert mesh_a is not None and mesh_b is not None
    assert isinstance(mesh_a, trimesh.Trimesh) and isinstance(mesh_b, trimesh.Trimesh) or\
        isinstance(mesh_a, trimesh.Scene) and isinstance(mesh_b, trimesh.Scene)
    return (
        all(trimesh.comparison.identifier_simple(mesh_a) == trimesh.comparison.identifier_simple(mesh_b)) or
        ((
               len(mesh_a.vertices.flatten()) == len(mesh_b.vertices.flatten()) and
               len(mesh_a.faces.flatten()) == len(mesh_b.faces.flatten())
        ) and (
               all(np.round(mesh_a.vertices, decimals=8).flatten() == np.round(mesh_b.vertices, decimals=8).flatten()) and
               all(mesh_a.faces.flatten() == mesh_b.faces.flatten())
        ))
    )