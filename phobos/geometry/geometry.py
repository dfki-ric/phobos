#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy

import numpy as np
import trimesh

from ..commandline_logging import get_logger
from ..defs import BPY_AVAILABLE
from ..io import representation
from ..utils.transform import create_transformation

if BPY_AVAILABLE:
    import bpy

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
    assert not oriented or isinstance(mesh, trimesh.Trimesh)
    # [TODO v2.1.0] Fix creation for Trimesh
    if isinstance(mesh, trimesh.Trimesh) or isinstance(mesh, trimesh.Scene):
        # scale the mesh
        mesh = deepcopy(mesh)
        mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))
        if oriented:
            extent = mesh.bounding_box_oriented.extents
            transform = mesh.bounding_box_oriented.primitive.transform
        else:
            extent = mesh.bounding_box.extents
            transform = mesh.bounding_box.primitive.transform
    elif BPY_AVAILABLE and isinstance(mesh, bpy.types.Object):
        extent = (np.max(mesh.bound_box, axis=0) - np.min(mesh.bound_box, axis=0))
        transform = np.identity(4)
        transform[0:3, 3] = np.average(mesh.bound_box, axis=0)
        # transform = np.array(mesh.matrix_local).dot(transform)
    else:
        raise ValueError(f"Received {type(mesh)}")

    return representation.Box(size=extent), transform


def create_sphere(mesh, scale=1.0):
    """ Create a sphere """
    # [TODO v2.1.0] Fix creation for Trimesh
    if isinstance(mesh, trimesh.Trimesh) or isinstance(mesh, trimesh.Scene):
        # scale the mesh
        mesh = deepcopy(mesh)
        mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))
        half_ext = mesh.bounding_box.extents
        transform = mesh.bounding_box.primitive.transform
    elif BPY_AVAILABLE and isinstance(mesh, bpy.types.Object):
        half_ext = np.max(mesh.bound_box, axis=0) - np.min(mesh.bound_box, axis=0)
        transform = np.identity(4)
        transform[0:3, 3] = np.average(mesh.bound_box, axis=0)
        # transform = np.array(mesh.matrix_local).dot(transform)
    else:
        raise ValueError(f"Received {type(mesh)}")

    r = np.sqrt(half_ext[0]**2 + half_ext[1]**2 + half_ext[2]**2)

    return representation.Sphere(radius=r * 0.5), transform


def create_cylinder(mesh, scale=1.0):
    """Create a cylinder.
    """
    # [TODO v2.1.0] Fix creation for Trimesh
    if isinstance(mesh, trimesh.Trimesh) or isinstance(mesh, trimesh.Scene):
        # scale the mesh
        mesh = deepcopy(mesh)
        mesh.apply_transform(np.diag((scale if type(scale) == list else [scale]*3) + [1]))
        c = mesh.bounding_cylinder
        transform = mesh.bounding_cylinder.primitive.transform
        # Find the length and the axis
        axis = mesh.bounding_cylinder.direction
        orthogonal = np.array(axis)
        if axis[0] != 0.0:
            orthogonal[0] = -axis[0]
        elif axis[1] != 0.0:
            orthogonal[1] = -axis[1]
        elif axis[2] != 0.0:
            orthogonal[2] = -axis[2]
        length = np.abs(c.direction).dot(c.extents)
        diameter = np.cross(axis, orthogonal).dot(c.extents)
    elif BPY_AVAILABLE:
        extent = (np.max(mesh.bound_box, axis=0) - np.min(mesh.bound_box, axis=0))
        deviation = np.abs(extent - np.average(extent))
        length = extent[np.argmax(deviation)]
        axis = np.argmax(deviation)
        diameter = np.max([extent[i] for i in range(3) if i != axis])
        rpy = [0, 0, 0]
        if axis == 0:
            rpy = [0, np.pi/2, 0]
        elif axis == 1:
            rpy = [np.pi/2, 0, 0]
        transform = create_transformation(xyz=np.average(mesh.bound_box, axis=0), rpy=rpy)
    else:
        raise ValueError(f"Received {type(mesh)}")

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
    out = (
        (
            len(mesh_a.vertices.flatten()) == len(mesh_b.vertices.flatten()) and
            len(mesh_a.faces.flatten()) == len(mesh_b.faces.flatten())
        ) and (
            all(np.round(mesh_a.vertices, decimals=8).flatten() == np.round(mesh_b.vertices, decimals=8).flatten()) and
            all(mesh_a.faces.flatten() == mesh_b.faces.flatten())
        )
    )
    try:
        trimesh_out = (
            all(trimesh.comparison.identifier_simple(mesh_a) == trimesh.comparison.identifier_simple(mesh_b))
        )
    except:
        # trimesh sometimes does utter sh** so we catch this here and assume false to be on the safe side
        trimesh_out = False
    return out or trimesh_out