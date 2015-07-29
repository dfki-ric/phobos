#!/usr/bin/python

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File inertia.py

Created on 13 Feb 2014
"""

import bpy
import mathutils
import phobos.defs as defs
import phobos.robotdictionary as robotdictionary
import phobos.utils.general as generalUtils
import phobos.utils.selection as selectionUtils
import phobos.utils.blender as blenderUtils
from phobos.logging import log


def register():
    """
    This function registers this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Registering inertia...")


def unregister():
    """
    This function unregisters this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Unregistering inertia...")


def calculateMassOfLink(link):
    """Calculates the masses of visual and collision objects found in a link,
    compares it to mass in link inertial object if present and returns the max of both, warning if they are not equal.

    :param link: The link you want to calculate the visuals and collision objects mass of.
    :type link: dict.
    :return: double.

    """
    objects = getInertiaRelevantObjects(link, ['visual', 'collision'])
    inertials = selectionUtils.getImmediateChildren(link, ['inertial'])
    objectsmass = selectionUtils.calculateSum(objects, 'mass')
    if len(inertials) == 1:
        inertialmass = inertials[0]['mass'] if 'mass' in inertials[0] else 0
    if objectsmass != inertialmass:
        log("Warning: Masses are inconsistent, sync masses of link!", "WARNING")
    return max(objectsmass, inertialmass)


def calculateInertia(mass, geometry):
    """Calculates the inertia of an object given its *geometry* and *mass* and
    returns the upper diagonal of the inertia 3x3 inertia tensor.

    :param mass: The objects mass.
    :type mass: double.
    :param geometry: The object dictionaries geometry part.
    :type geometry: dict.
    :return: tuple(6).

    """
    inertia = None
    gt = geometry['type']
    if gt == 'box':
        inertia = calculateBoxInertia(mass, geometry['size'])
    elif gt == 'cylinder':
        inertia = calculateCylinderInertia(mass, geometry['radius'], geometry['length'])
    elif gt == 'sphere':
        inertia = calculateSphereInertia(mass, geometry['radius'])
    elif gt == 'mesh':
        inertia = calculateEllipsoidInertia(mass, geometry['size'])
    return inertia


def calculateBoxInertia(mass, size):
    """Returns upper diagonal of inertia tensor of a box as tuple.

    :param mass: The box' mass.
    :type mass: double.
    :param size: The box' size.
    :type size: double.
    :return: tuple(6).

    """
    i = mass / 12
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateCylinderInertia(mass, r, h):
    """Returns upper diagonal of inertia tensor of a cylinder as tuple.

    :param mass: The cylinders mass.
    :type mass: double.
    :param r: The cylinders radius.
    :type r: double.
    :param h: The cylinders height.
    :type h: double.
    :return: tuple(6).

    """
    i = mass / 12 * (3 * r**2 + h**2)
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = 0.5 * mass * r**2
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateSphereInertia(mass, r):
    """Returns upper diagonal of inertia tensor of a sphere as tuple.

    :param mass: The spheres mass.
    :type mass: double.
    :param r: The spheres radius.
    :type r: double.
    :return: tuple(6).

    """
    i = 0.4 * mass * r**2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateEllipsoidInertia(mass, size):
    """Returns upper diagonal of inertia tensor of an ellipsoid as tuple.

    :param mass: The ellipsoids mass.
    :type mass: double.
    :param size: The ellipsoids size.
    :type r: double.
    :return: tuple(6).

    """
    i = mass / 5
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def inertiaListToMatrix(il):
    """Takes a tuple or list representing the upper diagonal of a 3x3 inertia tensor and returns the full tensor.

    :param il: The upper diagonal of a 3x3 inertia tensor
    :type il: tuple(6) or list[6].
    :return: blender matrix.

    """
    inertia = [[il[0], il[1], il[2]],
               [0.0, il[3], il[4]],
               [0.0, 0.0, il[5]]]
    return mathutils.Matrix(inertia)


def inertiaMatrixToList(im):
    """Takes a full 3x3 inertia tensor and returns a tuple representing the upper diagonal.

    :param im: The inertia tensor matrix.
    :type im: blender matrix
    :return: tuple(6)

    """
    return (im[0][0], im[0][1], im[0][2], im[1][1], im[1][2], im[2][2],)


def getInertiaRelevantObjects(link):
    """Returns a list of visual and collision objects of a link.
    If name-pairs of visual and collision objects are detected,
    the one with the latest change-date is used. If this is not clear,
    visual objects are omitted in favor of collision objects.

    :param link: The link you want to gather the inertia relevant objects for.
    :type link: Blender object.
    :return: list.

    """
    objdict = {obj.name: obj for obj in selectionUtils.getImmediateChildren(link, ['visual', 'collision'])}
    basenames = set()
    inertiaobjects = []
    for objname in objdict.keys():
        if 'mass' in objdict[objname]:
            if not objname.startswith('visual_') and not objname.startswith('collision_'):
                inertiaobjects.append(objdict[objname])
            else:
                basename = objname.replace(objdict[objname].phobostype + '_', '')
                if not basename in basenames:
                    basenames.add(basename)
                    collision = 'collision_'+basename if 'collision_'+basename in objdict.keys()\
                        and 'mass' in objdict['collision_'+basename] else None
                    visual = 'visual_'+basename if 'visual_'+basename in objdict.keys()\
                        and 'mass' in objdict['visual_'+basename] else None
                    if visual and collision:
                        try:
                            tv = generalUtils.datetimeFromIso(objdict[visual]['masschanged'])
                            tc = generalUtils.datetimeFromIso(objdict[collision]['masschanged'])
                            if tc < tv:  # if collision information is older than visual information
                                inertiaobjects.append(objdict[visual])
                            else:
                                inertiaobjects.append(objdict[collision])
                        except KeyError:  # if masschanged not present in both
                            inertiaobjects.append(objdict[collision])
                    else:
                        inertiaobjects.append(objdict[collision] if collision else objdict[visual])
    return inertiaobjects


def fuseInertiaData(inertials):
    """Returns mass, center of mass and inertia of a link as a whole, taking a list of inertials.

    *mass*: double.
    *com*: mathutils:Vector(3)
    *inertia*: mathutils:Matrix(3)

    :param inertials: The alist of objects relevant for the inertia of a link.
    :type inertials: list.
    :return: tuple(3) -- see description for content.

    """
    objects = []
    for o in inertials:
        objdict = None
        try:
            objdict = {'name': o.name,
                       'mass': o['mass'],
                       'com': o.matrix_local.to_translation(),
                       'rot': o.matrix_local.to_3x3(),
                       'inertia': o['inertia']
                       }
        except KeyError:
            print('Inertial object ' + o.name + ' is missing data.')
        if objdict:
            objects.append(objdict)
    if len(objects) > 0:
        mass, com, inertia = compound_inertia_analysis_3x3(objects)
        return mass, com, inertia
    else:
        return None, None, None


def createInertial(obj):
    """Creates an empty inertial object with the same world transform as the corresponding
    object and parents it to the correct link.

    :param obj: The object you want to copy the world transform from.
    :type obj: blender object.
    :return: blender object -- the newly created inertia.

    """
    if obj.phobostype == 'link':
        parent = obj
        size = (0.04, 0.04, 0.04)
    else:
        parent = obj.parent
        size = (0.02, 0.02, 0.02)
    rotation = obj.matrix_world.to_euler()
    center = obj.matrix_world.to_translation()
    inertial = blenderUtils.createPrimitive('inertial_' + obj.name, 'box', size,
                                   defs.layerTypes["inertial"], 'phobos_inertial', center, rotation)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    inertial.phobostype = 'inertial'
    bpy.ops.object.select_all(action="DESELECT")
    #utility.selectObjects([inertial], True, 0)

    selectionUtils.selectObjects([parent, inertial], True, 0)
    #bpy.context.scene.objects.active = parent.pose.bones[0]
    bpy.ops.object.parent_set(type='BONE_RELATIVE')
    return inertial


def createInertials(link, empty=False, preserve_children=False):
    # create inertial representations for visual and collision objects in link
    viscols = getInertiaRelevantObjects(link)
    # clean existing data
    if not preserve_children:
        oldinertials = selectionUtils.getImmediateChildren(link, ['inertial'])
    else:
        try:
            oldinertials = [bpy.data.objects['inertial_'+link.name]]
        except KeyError:
            oldinertials = None
    if oldinertials:
        selectionUtils.selectObjects(oldinertials, clear=True, active=0)
        bpy.ops.object.delete()
    if not preserve_children:
        for obj in viscols:
            if not empty:
                mass = obj['mass'] if 'mass' in obj else None
                geometry = robotdictionary.deriveGeometry(obj)
                if mass is not None:
                    inert = calculateInertia(mass, geometry)
                    if inert is not None:
                        inertial = createInertial(obj)
                        inertial['mass'] = mass
                        inertial['inertia'] = inert
            else:
                createInertial(obj)
    # compose inertial object for link
    if not empty:
        mass, com, inert = fuseInertiaData(selectionUtils.getImmediateChildren(link, ['inertial']))
        if mass and com and inert:
            inertial = createInertial(link)
            com_translate = mathutils.Matrix.Translation(com)
            inertial.matrix_local = com_translate
            bpy.ops.transform.translate(value=(0, 0, 0))  # FIXME: this is a trick to force Blender to apply matrix_local
            inertial['inertial/mass'] = mass
            inertial['inertial/inertia'] = inertiaMatrixToList(inert)
    else:
        createInertial(link)


################################################################################
# From here on we have code modified from Berti's implementation


def combine_cog_3x3(objects):
    """
    combine center the COG of a list of bodies given with their masses and their centers of gravity
    """
    if objects == []:
        return 0.0, mathutils.Vector((0.0,)*3)
    combined_com = mathutils.Vector((0.0,)*3)
    combined_mass = 0
    for obj in objects:
        combined_com = combined_com + obj['com'] * obj['mass']
        combined_mass += obj['mass']
    combined_com = combined_com / combined_mass
    return (combined_mass, combined_com)


def shift_cog_inertia_3x3(mass, cog, inertia_cog, ref_point=mathutils.Vector((0.0,)*3)):
    """
    shift inertia matrix, steiner theorem / parallel axis theorem, private method

    - without changing the orientation  -

    see SCISIC B.12 or featherstone 2.63, but not Selig (sign swap, not COG)

        c   = COG - O
        I_O = I_COG + m · c× (c× )T

        changed the formula to (Wikipedia):
        \mathbf{J} = \mathbf{I} + m \left[\left(\mathbf{R} \cdot \mathbf{R}\right) \mathbf{E}_{3} - \mathbf{R} \otimes \mathbf{R} \right],

        This was necessary as previous calculations founded on math libraries of cad2sim.
    """
    # diff_vec
    c               = cog - ref_point
    c_outer         = generalUtils.outerProduct(c, c)
    inertia_ref    = inertia_cog + mass * (c.dot(c) * mathutils.Matrix.Identity(3) - c_outer)

    return inertia_ref


def spin_inertia_3x3(inertia_3x3, rotmat, passive=True):
    """
    rotate the inertia matrix

    active and passive interpretation

        "passive"   -- the object stands still but the inertia is expressed with respect to a rotated reference frame
        "active"    -- the object moves and therefore its inertia

    consistent with 6x6 method :

         active     -- consistent with   N'  =  (H^T)^{-1}  *  N  *  H^{-1}
         passive    -- consistent with   N'  =  (H^T)       *  N  *  H

    WHERE IS a COMBINED METHOD of shifted and rotated inertia ? does it exist ?
    """
    R   = rotmat
    R_T = rotmat.transposed() #unlike .transpose(), this yields a new matrix and does not reset the original
    I   = inertia_3x3

    if passive :
        # the object stands still but the inertia is expressed with respect to a rotated reference frame
        rotated_inertia = R_T * I * R

    else:
        # the object moves and therefore its inertia
        rotated_inertia = R * I * R_T

    return rotated_inertia


def compound_inertia_analysis_3x3(objects):
    """
    Computes total mass, common center of mass and inertia matrix at CCOM
    """
    total_mass, common_cog = combine_cog_3x3(objects)

    shifted_inertias = list()
    for obj in objects:
        if not obj['rot'] == mathutils.Matrix.Identity(3): #if object is rotated in local space
            objinertia = spin_inertia_3x3(inertiaListToMatrix(obj['inertia']), obj['rot']) #transform inertia to link space
        else:
            objinertia = inertiaListToMatrix(obj['inertia']) # keep inertia
        inert_i_at_common_cog = shift_cog_inertia_3x3(obj['mass'], obj['com'], objinertia, common_cog)
        shifted_inertias.append(inert_i_at_common_cog)

    total_inertia_at_common_cog = mathutils.Matrix.Identity(3)
    total_inertia_at_common_cog.zero()
    for inertia in shifted_inertias:
        total_inertia_at_common_cog = total_inertia_at_common_cog + inertia

    return total_mass, common_cog, total_inertia_at_common_cog

