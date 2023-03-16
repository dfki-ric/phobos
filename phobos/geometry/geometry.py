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
    """
    Create a box element.
    """
    # scale the mesh
    mesh = deepcopy(mesh)
    mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))

    if oriented:
        half_ext = mesh.bounding_box_oriented.extents
        transform = mesh.bounding_box_oriented.transform
    else:
        half_ext = mesh.bounding_box.extents
        transform = mesh.bounding_box.transform

    return representation.Box(size=half_ext), transform


def create_sphere(mesh, scale=1.0):
    """ Create a sphere """
    # scale the mesh
    mesh = deepcopy(mesh)
    mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))

    half_ext = mesh.bounding_box.extents
    transform = mesh.bounding_box.transform

    r = np.amax(half_ext)

    return representation.Sphere(radius=r * 0.5), transform


def create_cylinder(mesh, scale=1.0):
    """Create a cylinder.
    """
    # scale the mesh
    mesh = deepcopy(mesh)
    mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))

    c = mesh.bounding_cylinder
    transform = mesh.bounding_cylinder.transform

    # Find the length and the axis
    axis = mesh.bounding_cylinder.direction
    orthogonal = axis
    if axis[0] != 0.0:
        orthogonal[0] = -axis[0]
    elif axis[1] != 0.0:
        orthogonal[1] = -axis[1]
    elif axis[2] != 0.0:
        orthogonal[2] = -axis[2]
    length = np.abs(c.direction).dot(c.extents)
    diameter = np.cross(axis, orthogonal).dot(c.extents)

    return representation.Cylinder(radius=diameter/2, length=length), transform


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