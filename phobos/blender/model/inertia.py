#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains all functions to model inertias within Blender.
"""

import mathutils
import numpy
import numpy as np

from .. import reserved_keys
from ..phoboslog import log
from ..utils import general as gUtils
from ..utils import selection as sUtils
from ..utils.io import getExpSettings
from ..utils.validation import validate, validateInertiaData

from ...io import representation
from ...utils.inertia import calculateBoxInertia, calculateCylinderInertia, calculateSphereInertia, \
    calculateMeshInertia


@validate('geometry_type')
def calculateInertia(obj, mass, geometry, errors=None, adjust=False, logging=False):
    """Calculates the inertia of an object using the specified mass and
       optionally geometry.

    Args:
      obj(bpy.types.Object): object to calculate inertia from
      mass(float): mass of object
      geometry(representation.Geometry instance, optional): geometry part of the object dictionary
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

    # Get the rotation of the object
    object_rotation = np.array(obj.rotation_euler.to_matrix())

    if isinstance(geometry, representation.Box):
        inertia = calculateBoxInertia(mass, geometry.size)
    elif isinstance(geometry, representation.Cylinder):
        inertia = calculateCylinderInertia(mass, geometry.radius, geometry.length)
    elif isinstance(geometry, representation.Sphere):
        inertia = calculateSphereInertia(mass, geometry.radius)
    elif isinstance(geometry, representation.Mesh):
        sUtils.selectObjects((obj,), clear=True, active=0)
        inertia = calculateMeshInertia(mass, obj.data, scale=obj.scale)

    # Correct the inertia orientation to account for Cylinder / mesh orientation issues
    inertia = object_rotation.dot(inertiaListToMatrix(inertia)).dot(object_rotation.transpose())

    return inertiaMatrixToList(inertia)


def inertiaListToMatrix(inertialist):
    """Transforms a list (upper diagonal of a 3x3 tensor) and returns a full tensor matrix.

    Args:
      inertialist(tuple(6): the upper diagonal of a 3x3 inertia tensor

    Returns:
      : mathutils.Matrix -- full tensor matrix generated from the list

    """
    il = inertialist
    inertia = [[il[0], il[1], il[2]], [il[1], il[3], il[4]], [il[2], il[4], il[5]]]
    return np.array(inertia)


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


    expsetting = 1e-10

    # Find objects who have some inertial data
    inertials = [obj for obj in inertials if any([key in reserved_keys.INERTIAL_KEYS for key in obj.keys()])]

    # Check for an empty list -> No inertials to fuse
    if not inertials:
        return 1e-3, [0.0, 0.0, 0.0], numpy.diag([1e-3, 1e-3, 1e-3])

    fused_inertia = numpy.zeros((3, 3))
    fused_com = numpy.zeros((1, 3))
    fused_mass = 0.0

    # Calculate the fused mass and center of mass
    fused_mass, fused_com = combine_com_3x3(inertials)

    # Check for conformity
    if fused_mass <= expsetting:
        log(" Correcting fused mass : negative semidefinite value.", 'WARNING')
        fused_mass = expsetting if fused_mass < expsetting else fused_mass

    # TODO Maybe we can reuse the functions defined here.
    # Calculate the fused inertias
    for obj in inertials:
        # Get the rotation of the inertia
        current_Rotation = numpy.array(obj.matrix_local.to_3x3())
        current_Inertia = numpy.array(inertiaListToMatrix(obj['inertia']))
        # Rotate the inertia into the current frame
        current_Inertia = numpy.dot(
            numpy.dot(current_Rotation.T, current_Inertia), current_Rotation
        )
        # Move the inertia to the center of mass
        # Get the current relative position of the center of mass
        relative_position = numpy.array(obj.matrix_local.translation) - fused_com
        # Calculate the translational influence
        current_Inertia += obj['mass'] * (
            relative_position.T * relative_position * numpy.eye(3)
            - numpy.outer(relative_position, relative_position)
        )
        fused_inertia += numpy.dot(numpy.dot(current_Rotation.T, current_Inertia), current_Rotation)

    # Check the inertia
    if any(element <= expsetting for element in fused_inertia.diagonal()):
        log(" Correting fused inertia : negative semidefinite diagonal entries.", 'WARNING')
        for i in range(3):
            fused_inertia[i, i] = expsetting if fused_inertia[i, i] <= expsetting else fused_inertia[i, i]

    if any(element <= expsetting for element in numpy.linalg.eigvals(fused_inertia)):
        log(" Correcting fused inertia : negative semidefinite eigenvalues", 'WARNING')
        S, V = numpy.linalg.eig(fused_inertia)
        S[S <= expsetting] = expsetting
        fused_inertia = V.dot(numpy.diag(S).dot(V.T))
        # Add minimum value for the inertia
        for i in range(3):
            fused_inertia[i, i] += expsetting if fused_inertia[i, i] <= expsetting else fused_inertia[i, i]

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
        combined_com = combined_com + obj.matrix_local.translation * obj['mass']
        combined_mass += obj['mass']
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
        rotated_inertia = R_T @ I @ R

    else:
        # the object moves and therefore its inertia
        rotated_inertia = R @ I @ R_T

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
