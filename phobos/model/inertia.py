#!/usr/bin/python
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------------

"""
Contains all functions to model inertias within Blender.
"""

import math
import numpy
import bpy
import mathutils
import phobos.defs as defs
from phobos.phoboslog import log
import phobos.utils.general as gUtils
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
import phobos.utils.blender as bUtils
import phobos.utils.naming as nUtils
from phobos.model.geometries import deriveGeometry
from phobos.model.poses import deriveObjectPose
from phobos.utils.validation import validate


@validate('inertia_data')
def createInertial(inertialdict, obj, size=0.03, errors=None, adjust=False, logging=False):
    """Creates the Blender representation of a given inertial provided a dictionary.

    Args:
      inertialdict(dict): intertial data
      obj:
      size: (Default value = 0.03)
      errors: (Default value = None)
      adjust: (Default value = False)
      logging: (Default value = False)

    Returns:
      : bpy_types.Object -- newly created blender inertial object

    """
    if errors and not adjust:
        log('Can not create inertial object.', 'ERROR')

    try:
        origin = mathutils.Vector(inertialdict['pose']['translation'])
    except KeyError:
        origin = mathutils.Vector()

    # create new inertial object
    name = nUtils.getUniqueName('inertial_' + nUtils.getObjectName(obj), bpy.data.objects)
    inertialobject = bUtils.createPrimitive(
        name,
        'box',
        (size,) * 3,
        defs.layerTypes["inertial"],
        pmaterial='phobos_inertial',
        phobostype='inertial',
    )
    sUtils.selectObjects((inertialobject,), clear=True, active=0)
    bpy.ops.object.transform_apply(scale=True)

    # set position according to the parent link
    inertialobject.matrix_world = obj.matrix_world
    parent = obj
    if parent.phobostype != 'link':
        parent = sUtils.getEffectiveParent(obj, ignore_selection=True)
    eUtils.parentObjectsTo(inertialobject, parent)

    # position and parent the inertial object relative to the link
    #inertialobject.matrix_local = mathutils.Matrix.Translation(origin)
    sUtils.selectObjects((inertialobject,), clear=True, active=0)
    #bpy.ops.object.transform_apply(scale=True)

    # add properties to the object
    for prop in ('mass', 'inertia'):
        inertialobject['inertial/' + prop] = inertialdict[prop]
    return inertialobject


@validate('geometry_type')
def calculateInertia(obj, mass, geometry_dict=None, errors=None, adjust=False, logging=False):
    """Calculates the inertia of an object using the specified mass and
       optionally geometry.

    Args:
      obj(bpy.types.Object): object to calculate inertia from
      mass(float): mass of object
      geometry_dict(dict, optional): geometry part of the object dictionary
    Returns(tuple): (Default value = None)
      geometry_dict(dict, optional): geometry part of the object dictionary
    Returns(tuple):
    tuple(6) of upper diagonal of the inertia 3x3 tensor (Default value = None)
      errors: (Default value = None)
      adjust: (Default value = False)
      logging: (Default value = False)

    Returns:

    """
    if errors and not adjust:
        if logging:
            log("Can not calculate inertia from object.", 'ERROR')
        return None

    inertia = None
    if not geometry_dict:
        geometry = deriveGeometry(obj)

    # Get the rotation of the object
    object_rotation = obj.rotation_euler.to_matrix()

    if geometry['type'] == 'box':
        inertia = calculateBoxInertia(mass, geometry['size'])
    elif geometry['type'] == 'cylinder':
        inertia = calculateCylinderInertia(mass, geometry['radius'], geometry['length'])
    elif geometry['type'] == 'sphere':
        inertia = calculateSphereInertia(mass, geometry['radius'])
    elif geometry['type'] == 'mesh':
        sUtils.selectObjects((obj,), clear=True, active=0)
        inertia = calculateMeshInertia(mass, obj.data)

    # Correct the inertia orientation to account for Cylinder / mesh orientation issues
    inertia = object_rotation *inertiaListToMatrix(inertia) *object_rotation.transposed()

    return inertiaMatrixToList(inertia)


def calculateBoxInertia(mass, size):
    """Returns upper diagonal of inertia tensor of a box as tuple.

    Args:
      mass(float): The box' mass.
      size(iterable): The box' size.

    Returns:
      : tuple(6)

    """
    i = mass / 12
    ixx = i * (size[1] ** 2 + size[2] ** 2)
    ixy = 0
    ixz = 0
    iyy = i * (size[0] ** 2 + size[2] ** 2)
    iyz = 0
    izz = i * (size[0] ** 2 + size[1] ** 2)
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateCylinderInertia(mass, r, h):
    """Returns upper diagonal of inertia tensor of a cylinder as tuple.

    Args:
      mass(float): The cylinders mass.
      r(float): The cylinders radius.
      h(float): The cylinders height.

    Returns:
      : tuple(6)

    """
    i = mass / 12 * (3 * r ** 2 + h ** 2)
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = 0.5 * mass * r ** 2
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateSphereInertia(mass, r):
    """Returns upper diagonal of inertia tensor of a sphere as tuple.

    Args:
      mass(float): The spheres mass.
      r(float): The spheres radius.

    Returns:
      : tuple(6)

    """
    i = 0.4 * mass * r ** 2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateEllipsoidInertia(mass, size):
    """Returns upper diagonal of inertia tensor of an ellipsoid as tuple.

    Args:
      mass(float): The ellipsoids mass.
      size: The ellipsoids size.

    Returns:
      : tuple(6)

    """
    i = mass / 5
    ixx = i * (size[1] ** 2 + size[2] ** 2)
    ixy = 0
    ixz = 0
    iyy = i * (size[0] ** 2 + size[2] ** 2)
    iyz = 0
    izz = i * (size[0] ** 2 + size[1] ** 2)
    return ixx, ixy, ixz, iyy, iyz, izz


def calculateMeshInertia(mass, data):
    """Calculates and returns the inertia tensor of arbitrary mesh objects.

    Implemented after the general idea of 'Finding the Inertia Tensor of a 3D Solid Body,
    Simply and Quickly' (2004) by Jonathan Blow (1) with formulas for tetrahedron inertia
    from 'Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its
    Vertex Coordinates' (2004) by F. Tonon. (2)

    Links: (1) http://number-none.com/blow/inertia/body_i.html
           (2) http://docsdrive.com/pdfs/sciencepublications/jmssp/2005/8-11.pdf

    Args:
      data(bpy.types.BlendData): mesh data of the object
      mass(float): mass of the object

    Returns:
      6: inertia tensor

    """
    tetrahedra = []
    mesh_volume = 0
    origin = mathutils.Vector((0.0, 0.0, 0.0))

    vertices = data.vertices
    prev_mode = bpy.context.mode
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.quads_convert_to_tris(quad_method='FIXED')
    bpy.ops.object.mode_set(mode=prev_mode)
    polygons = data.polygons

    for triangle in polygons:
        verts = [vertices[index].co for index in triangle.vertices]

        tri_normal = triangle.normal
        tri_centre = triangle.center
        ref_tri_vector = tri_centre - origin
        # angle returns the angle in radians
        normal_angle = ref_tri_vector.angle(tri_normal, math.pi / 2.0)

        sign = -1 if normal_angle > math.pi / 2.0 else 1

        J = mathutils.Matrix(
            (
                (verts[0][0], verts[0][1], verts[0][2], 1),
                (verts[1][0], verts[1][1], verts[1][2], 1),
                (verts[2][0], verts[2][1], verts[2][2], 1),
                (origin[0], origin[1], origin[2], 1),
            )
        )

        abs_det_J = J.determinant()

        volume = sign * abs_det_J / 6

        com = mathutils.Vector(
            (
                (verts[0][0] + verts[1][0] + verts[2][0] + origin[0]) / 4,
                (verts[0][1] + verts[1][1] + verts[2][1] + origin[1]) / 4,
                (verts[0][2] + verts[1][2] + verts[2][2] + origin[2]) / 4,
            )
        )

        tetrahedra.append({'sign': sign, 'abs(det(J))': abs_det_J, 'J': J, 'centre_of_mass': com})
        mesh_volume += volume

    density = mass / mesh_volume
    i = mathutils.Matrix().to_3x3()
    i.zero()
    for tetrahedron in tetrahedra:

        J = tetrahedron['J']
        x1 = J[0][0]
        y1 = J[0][1]
        z1 = J[0][2]
        x2 = J[1][0]
        y2 = J[1][1]
        z2 = J[1][2]
        x3 = J[2][0]
        y3 = J[2][1]
        z3 = J[2][2]
        x4 = J[3][0]
        y4 = J[3][1]
        z4 = J[3][2]

        abs_det_J = tetrahedron['abs(det(J))']
        sign = tetrahedron['sign']

        # TODO this might be easier with numpy (and more beautiful)
        a = (
            sign
            * density
            * abs_det_J
            * (
                y1 ** 2
                + y1 * y2
                + y2 ** 2
                + y1 * y3
                + y2 * y3
                + y3 ** 2
                + y1 * y4
                + y2 * y4
                + y3 * y4
                + y4 ** 2
                + z1 ** 2
                + z1 * z2
                + z2 ** 2
                + z1 * z3
                + z2 * z3
                + z3 ** 2
                + z1 * z4
                + z2 * z4
                + z3 * z4
                + z4 ** 2
            )
            / 60
        )

        b = (
            sign
            * density
            * abs_det_J
            * (
                x1 ** 2
                + x1 * x2
                + x2 ** 2
                + x1 * x3
                + x2 * x3
                + x3 ** 2
                + x1 * x4
                + x2 * x4
                + x3 * x4
                + x4 ** 2
                + z1 ** 2
                + z1 * z2
                + z2 ** 2
                + z1 * z3
                + z2 * z3
                + z3 ** 2
                + z1 * z4
                + z2 * z4
                + z3 * z4
                + z4 ** 2
            )
            / 60
        )

        c = (
            sign
            * density
            * abs_det_J
            * (
                x1 ** 2
                + x1 * x2
                + x2 ** 2
                + x1 * x3
                + x2 * x3
                + x3 ** 2
                + x1 * x4
                + x2 * x4
                + x3 * x4
                + x4 ** 2
                + y1 ** 2
                + y1 * y2
                + y2 ** 2
                + y1 * y3
                + y2 * y3
                + y3 ** 2
                + y1 * y4
                + y2 * y4
                + y3 * y4
                + y4 ** 2
            )
            / 60
        )

        a_bar = (
            sign
            * density
            * abs_det_J
            * (
                2 * y1 * z1
                + y2 * z1
                + y3 * z1
                + y4 * z1
                + y1 * z2
                + 2 * y2 * z2
                + y3 * z2
                + y4 * z2
                + y1 * z3
                + y2 * z3
                + 2 * y3 * z3
                + y4 * z3
                + y1 * z4
                + y2 * z4
                + y3 * z4
                + 2 * y4 * z4
            )
            / 120
        )

        b_bar = (
            sign
            * density
            * abs_det_J
            * (
                2 * x1 * z1
                + x2 * z1
                + x3 * z1
                + x4 * z1
                + x1 * z2
                + 2 * x2 * z2
                + x3 * z2
                + x4 * z2
                + x1 * z3
                + x2 * z3
                + 2 * x3 * z3
                + x4 * z3
                + x1 * z4
                + x2 * z4
                + x3 * z4
                + 2 * x4 * z4
            )
            / 120
        )

        c_bar = (
            sign
            * density
            * abs_det_J
            * (
                2 * x1 * y1
                + x2 * y1
                + x3 * y1
                + x4 * y1
                + x1 * y2
                + 2 * x2 * y2
                + x3 * y2
                + x4 * y2
                + x1 * y3
                + x2 * y3
                + 2 * x3 * y3
                + x4 * y3
                + x1 * y4
                + x2 * y4
                + x3 * y4
                + 2 * x4 * y4
            )
            / 120
        )

        i += inertiaListToMatrix([a, -b_bar, -c_bar, b, -a_bar, c])

    return i[0][0], i[0][1], i[0][2], i[1][1], i[1][2], i[2][2]


def inertiaListToMatrix(inertialist):
    """Transforms a list (upper diagonal of a 3x3 tensor) and returns a full tensor matrix.

    Args:
      inertialist(tuple(6): the upper diagonal of a 3x3 inertia tensor

    Returns:
      : mathutils.Matrix -- full tensor matrix generated from the list

    """
    il = inertialist
    inertia = [[il[0], il[1], il[2]], [il[1], il[3], il[4]], [il[2], il[4], il[5]]]
    return mathutils.Matrix(inertia)


def inertiaMatrixToList(im):
    """Takes a full 3x3 inertia tensor as a matrix and returns a tuple representing
    the upper diagonal.

    Args:
      im(mathutil.Matrix): The inertia tensor matrix.

    Returns:
      : tuple(6)

    """
    return im[0][0], im[0][1], im[0][2], im[1][1], im[1][2], im[2][2]


def fuse_inertia_data(inertials):
    """Computes combined mass, center of mass and inertia given a list of inertial objects.
    Computation based on Modern Robotics, Lynch & Park, p. 287 .

    If no inertials are found (None, None, None) is returned.

    If successful, the tuple contains this information:
        *mass*: float
        *com*: mathutils.Vector(3)
        *inertia*: mathutils.Matrix(3)

    Args:
      inertials(list): the alist of objects relevant for the inertia of a link

    Returns:
      3: tuple of mass, COM and inertia or None(3) if no inertials are found

    """

    # Find objects who have some inertial data
    for obj in inertials:
        if not any([True for key in obj.keys() if key.startswith('inertial/')]):
            inertials.remove(obj)

    # Check for an empty list -> No inertials to fuse
    if not inertials:
        return None, None, None

    fused_inertia = numpy.zeros((3,3))
    fused_com = numpy.zeros((1,3))
    fused_mass = 0.0

    # Calculate the fused mass and center of mass
    fused_mass, fused_com = combine_com_3x3(inertials)

    # Check for conformity
    if fused_mass <= 0.0:
        log(" Correcting fused mass : negative semidefinite value.", 'WARNING')
        fused_mass = 1e-3 if fused_mass < 1e-3 else fused_mass

    # TODO Maybe we can reuse the functions defined here.
    # Calculate the fused inertias
    for obj in inertials:
        # Get the rotation of the inertia
        current_Rotation = numpy.array(obj.matrix_local.to_3x3())
        current_Inertia = numpy.array(inertiaListToMatrix(obj['inertial/inertia']))
        # Rotate the inertia into the current frame
        current_Inertia = numpy.dot(numpy.dot(current_Rotation.T, current_Inertia ), current_Rotation)
        # Move the inertia to the center of mass
        # Get the current relative position of the center of mass
        relative_position = numpy.array(obj.matrix_local.translation) - fused_com
        # Calculate the translational influence
        current_Inertia += obj['inertial/mass']*(relative_position.T* relative_position *numpy.eye(3) - numpy.outer(relative_position,relative_position))
        fused_inertia += numpy.dot(numpy.dot(current_Rotation.T, current_Inertia ), current_Rotation)

    # Check the inertia
    if any(element <= 0.0 for element in fused_inertia.diagonal()):
        log(" Correting fused inertia : negative semidefinite diagonal entries.", 'WARNING')
        for i in range(3):
            fused_inertia[i,i] = 1e-3 if fused_inertia[i,i] <= 1e-3 else fused_inertia[i,i]

    if any(element <= 0.0 for element in numpy.linalg.eigvals(fused_inertia)):
        log(" Correcting fused inertia : negative semidefinite eigenvalues", 'WARNING')
        U, S, V = numpy.linalg.svd(fused_inertia)
        S[S <= 0.0] = 1e-3
        fused_inertia = U * S * V


    return fused_mass, fused_com, fused_inertia




def combine_com_3x3(objects):
    """Combines center of mass (COM) of a list of bodies given their masses and COMs.
    This code was adapted from an implementation generously provided by Bertold Bongardt.

    Args:
      objects(list containing phobos dicts): The list of objects you want to combine the COG.

    Returns:

    """
    if not objects:
        log("No proper object list...", 'DEBUG')
        return 0.0, mathutils.Vector((0.0,) * 3)
    combined_com = mathutils.Vector((0.0,) * 3)
    combined_mass = 0
    for obj in objects:
        combined_com = combined_com + obj.matrix_local.translation * obj['inertial/mass']
        combined_mass += obj['inertial/mass']
    combined_com = combined_com / combined_mass
    log("  Combined center of mass: " + str(combined_com), 'DEBUG')
    return combined_mass, combined_com


def shift_com_inertia_3x3(mass, com, inertia_com, ref_point=mathutils.Vector((0.0,) * 3)):
    """Shifts the center of mass of a 3x3 inertia.

    This code was adapted from an implementation generously provided by Bertold Bongardt.

    TODO cleanup docstring

    shift inertia matrix, steiner theorem / parallel axis theorem, private method

    - without changing the orientation  -

    see SCISIC B.12 or featherstone 2.63, but not Selig (sign swap, not COG)

    | c   = COG - O
    | I_O = I_COG + m · c× (c× )T
    |
    | changed the formula to (Wikipedia):
    | \\mathbf{J} = \\mathbf{I} + m \\left[\\left(\\mathbf{R} \\cdot \\mathbf{R}\\right)
    | \\mathbf{E}_{3} - \\mathbf{R} \\otimes \\mathbf{R} \\right],

    This was necessary as previous calculations founded on math libraries of cad2sim.

    Args:
      mass:
      com:
      inertia_com:
      ref_point: (Default value = mathutils.Vector((0.0)
      ) * 3):

    Returns:

    """
    c = com - ref_point  # difference vector
    c_outer = gUtils.outerProduct(c, c)
    inertia_ref = inertia_com + mass * (c.dot(c) * mathutils.Matrix.Identity(3) - c_outer)

    return inertia_ref


def spin_inertia_3x3(inertia_3x3, rotmat, passive=True):
    """Rotates an inertia matrix.

    active and passive interpretation

    passive
        the object stands still but the inertia is expressed with respect to a rotated reference
        frame

    active
        object moves and therefore its inertia

    consistent with 6x6 method :

    active
        consistent with   N'  =  (H^T)^{-1}  *  N  *  H^{-1}

    passive
        consistent with   N'  =  (H^T)       *  N  *  H

    WHERE IS a COMBINED METHOD of shifted and rotated inertia ? does it exist ?

    Args:
      inertia_3x3:
      rotmat:
      passive: (Default value = True)

    Returns:

    """
    # DOCU improve this docstring
    R = rotmat
    # unlike transpose(), this yields a new matrix and does not reset the original
    R_T = rotmat.transposed()
    I = inertia_3x3

    if passive:
        # the object stands still but the inertia is expressed with respect to a rotated reference frame
        rotated_inertia = R_T * I * R

    else:
        # the object moves and therefore its inertia
        rotated_inertia = R * I * R_T

    return rotated_inertia


def compound_inertia_analysis_3x3(objects):
    """Computes total mass, common center of mass and inertia matrix at CCOM

    Args:
      objects:

    Returns:

    """
    total_mass, common_com = combine_com_3x3(objects)

    shifted_inertias = list()
    for obj in objects:
        if 'rot' in obj and not obj['rot'] == mathutils.Matrix.Identity(
            3
        ):  # if object is rotated in local space
            objinertia = spin_inertia_3x3(
                inertiaListToMatrix(obj['inertia']), obj['rot']
            )  # transform inertia to link space
        else:
            objinertia = inertiaListToMatrix(obj['inertia'])  # keep inertia
        inert_i_at_common_com = shift_com_inertia_3x3(
            obj['mass'], obj['com'], objinertia, common_com
        )
        shifted_inertias.append(inert_i_at_common_com)

    total_inertia_at_common_com = mathutils.Matrix.Identity(3)
    total_inertia_at_common_com.zero()
    for inertia in shifted_inertias:
        total_inertia_at_common_com = total_inertia_at_common_com + inertia

    return total_mass, common_com, total_inertia_at_common_com


def gatherInertialChilds(obj, objectlist):
    """Gathers recursively all inertial object children from the specified object.

    The inertia objects need to be in the specified objectlist to be encluded. Also, links which
    are not in the objectlist, will be considered, too. This will gather inertial objects which are
    child of a link *not* in the list.

    Args:
      obj(bpy.types.Object): object to start the recursion (preferably a link)
      objectlist(list(bpy.types.Object): objects to consider for the recursion

    Returns:
      : list(bpy.types.Object) -- inertial objects which belong to the specified obj

    """
    from phobos.utils.validation import validateInertiaData

    # only gather the links that are not in the list
    childlinks = [
        link for link in obj.children if link.phobostype == 'link' and link not in objectlist
    ]
    # only gathe the inertials that are in the list
    inertialobjs = [
        inert for inert in obj.children if inert.phobostype == 'inertial' and inert in objectlist
    ]

    inertials = []
    for inertial in inertialobjs:
        # log inertials with errors
        errors, *_ = validateInertiaData(inertial, adjust=True)
        if errors:
            for error in errors:
                error.log()

        # add inertial to list
        inertials.append(inertial)

    # collect optional inertials in sublinks
    for link in childlinks:
        if link.children:
            inertials.extend(gatherInertialChilds(link, objectlist=objectlist))

    return inertials
