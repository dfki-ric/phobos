#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import os
import numpy
import trimesh
import numpy as np

from . import io
from ..io import representation
from ..utils import misc
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)


def round_vector(v):
    return round(v[0], 6), round(v[1], 6), round(v[2], 6)


def get_vertex_id(x, mesh):
    return np.logical_and(
        np.logical_and(
            mesh.vertices[:, 0] == x[0],
            mesh.vertices[:, 1] == x[1]),
        mesh.vertices[:, 2] == x[2]
    ).nonzero()[0][0]


def create_box(mesh, oriented=True, scale=1.0):
    """Create a box element.
    """

    if oriented:
        half_ext = mesh.bounding_box_oriented.primitive.extents
    else:
        half_ext = mesh.bounding_box.primitive.extents

    half_ext = numpy.array(half_ext) * scale

    return representation.Box(size=half_ext)


def create_sphere(mesh, oriented=True, scale=1.0):
    """ Create a sphere """

    if oriented:
        half_ext = mesh.bounding_box_oriented.primitive.extents
    else:
        half_ext = mesh.bounding_box.primitive.extents

    r = numpy.amax(half_ext)

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


def create_convex_hull(mesh, filepath, urdf_path, scale=1.0):
    """ Replace the mesh with its convex hull. Exports the mesh as well.
    """
    convex_mesh = mesh.convex_hull
    if "_convexhull" in filepath and all(
            trimesh.comparison.identifier_simple(mesh) == trimesh.comparison.identifier_simple(mesh.convex_hull)):
        log.info(f"Mesh is already convex hull: {filepath}")
        return representation.Mesh(filename=filepath, scale=[scale, scale, scale])

    # Get the format and export
    filename, file_extension = os.path.splitext(filepath)
    filename += '_convexhull' + file_extension

    io.export_mesh(convex_mesh, filename, urdf_path=urdf_path)

    return representation.Mesh(filename=filename, scale=[scale, scale, scale])


def get_reflection_matrix(point=numpy.array((0, 0, 0)), normal=numpy.array((0, 1, 0))):
    return trimesh.transformations.reflection_matrix(point, normal)


def mirror_geometry(element, urdf_path, transform=None, name_replacements=None):
    """ Replace the mesh with its convex hull. Exports the mesh as well.
    """

    if urdf_path and os.path.isfile(urdf_path):
        urdf_path = os.path.split(urdf_path)[0]

    # Get the mesh
    if element is None:
        return

    if not hasattr(element.geometry, 'filename'):
        return

    mesh = io.import_mesh(element.geometry.filename, urdf_path=urdf_path)

    if transform is None:
        transform = get_reflection_matrix()
    mirrored_mesh = mesh.apply_transform(transform)

    # Get the format and export
    filename, file_extension = os.path.splitext(element.geometry.filename)
    if name_replacements is None:
        filename += '_mirrored' + file_extension
    else:
        new_filename = os.path.join(
            os.path.dirname(filename),
            misc.regex_replace(os.path.basename(filename), name_replacements))
        if new_filename == filename:
            new_filename += "_mirrored"
        filename = new_filename + file_extension

    io.export_mesh(mirrored_mesh, filename, urdf_path=urdf_path)

    element.geometry = representation.Mesh(filename=filename, scale=[1.0, 1.0, 1.0])


def has_enough_vertices(element, urdf_path):
    # Get the collison mesh
    if element is None:
        return False
    if not hasattr(element.geometry, 'filename'):
        return True
    mesh = io.import_mesh(element.geometry.filename, urdf_path=urdf_path)
    if mesh is None:
        return False
    zero_transform = element.origin.to_matrix()
    mesh.apply_transform(zero_transform)
    if len(mesh.vertices) <= 3:
        log.info(f"Mesh has less than 4 vertices: {element.geometry._filename}")
        return False
    elif np.linalg.norm(mesh.bounding_box_oriented.primitive.extents) <= 0.01:
        log.info(f"Mesh is smaller than 1cm: {element.geometry._filename}")
        return False
    return True


def replace_geometry(element, urdf_path, shape='box', oriented=False, scale=1.0):
    """Replace the geometry of the element with an oriented shape. urdf_path is needed for mesh loading.
    """
    if type(element) is list:
        return [replace_geometry(e, urdf_path, shape, oriented, scale) for e in element]
    if urdf_path and os.path.isfile(urdf_path):
        urdf_path = os.path.split(urdf_path)[0]
    # Get the collison mesh
    if element is None:
        return

    if not hasattr(element.geometry, 'filename'):
        return

    mesh = io.import_mesh(element.geometry.filename, urdf_path=urdf_path)
    zero_transform = element.origin.to_matrix()
    mesh.apply_transform(zero_transform)

    if shape == 'sphere':
        new_shape = create_sphere(mesh, oriented=oriented, scale=scale)
    elif shape == 'cylinder':
        new_shape = create_cylinder(mesh, oriented=oriented, scale=scale)
    elif shape == 'box':
        new_shape = create_box(mesh, oriented=oriented, scale=scale)
    elif shape == 'convex':
        new_shape = create_convex_hull(mesh, element.geometry.filename, urdf_path)
    else:
        raise Exception('Shape {} not implemented. Please choose sphere, cylinder, box or convex.'.format(shape))

    if oriented and not shape == 'convex':
        rel_transform = mesh.bounding_box_oriented.primitive.transform
    elif not shape == 'convex':
        rel_transform = mesh.bounding_box.primitive.transform
    else:
        rel_transform = numpy.eye(4)

    new_origin = representation.Pose.from_matrix(rel_transform)

    element.geometry = new_shape
    if isinstance(element.geometry, representation.Mesh):
        element.geometry.scale = [scale] * 3
    element.origin = new_origin
    return


def improve_mesh(mesh):
    mesh.remove_duplicate_faces()
    mesh.remove_infinite_values()
    mesh.remove_unreferenced_vertices()
    return mesh


def reduce_mesh(mesh, factor):
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
    return (
        all(trimesh.comparison.identifier_simple(mesh_a) == trimesh.comparison.identifier_simple(mesh_b)) or
        ((
               len(mesh_a.vertices.flatten()) == len(mesh_b.vertices.flatten()) and
               len(mesh_a.faces.flatten()) == len(mesh_b.faces.flatten()) and
               len(mesh_a.edges.flatten()) == len(mesh_b.edges.flatten())
        ) and (
               all(np.round(mesh_a.vertices, decimals=8).flatten() == np.round(mesh_b.vertices, decimals=8).flatten()) and
               all(mesh_a.faces.flatten() == mesh_b.faces.flatten()) and
               all(mesh_a.edges.flatten() == mesh_b.edges.flatten())
        ))
    )