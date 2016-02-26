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
import math
import phobos.defs as defs
import phobos.utils.general as generalUtils
import phobos.utils.selection as selectionUtils
import phobos.utils.blender as blenderUtils
import phobos.utils.naming as namingUtils
from phobos.logging import log


def register():
    """This function is called when this module is registered to blender.

    """
    print("Registering inertia...")


def unregister():
    """This function is called when this module is unregistered from blender.

    """
    print("Unregistering inertia...")


def calculateMassOfLink(link):
    """Calculates the masses of visual and collision objects found in a link,
    compares it to mass in link inertial object if present and returns the max of both, warning if they are not equal.

    :param link: The link you want to calculate the visuals and collision objects mass of.
    :type link: dict
    :return: double

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
    :type mass: double
    :param geometry: The object dictionaries geometry part.
    :type geometry: dict
    :return: tuple(6)

    """
    inertia = None
    gt = geometry['type']
    if gt == 'box':
        inertia = calculateBoxInertia(mass, geometry['size'])
    elif gt == 'cylinder':
        inertia = calculateCylinderInertia(mass, geometry['radius'], geometry['length'])
    elif gt == 'sphere':
        inertia = calculateSphereInertia(mass, geometry['radius'])
    elif gt == 'capsule':
        inertia = calculateCapsuleInertia(mass, geometry['radius'], geometry['length'])
    #elif gt == 'mesh':
    #    inertia = calculateEllipsoidInertia(mass, geometry['size'])
    return inertia


def calculateBoxInertia(mass, size):
    """Returns upper diagonal of inertia tensor of a box as tuple.

    :param mass: The box' mass.
    :type mass: double
    :param size: The box' size.
    :type size: double
    :return: tuple(6)

    """
    print('###########################')
    print('calculating box inertia')
    print('###########################')

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
    :type mass: double
    :param r: The cylinders radius.
    :type r: double
    :param h: The cylinders height.
    :type h: double
    :return: tuple(6)

    """
    print('###########################')
    print('calculating cylinder inertia')
    print('###########################')

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
    :type mass: double
    :param r: The spheres radius.
    :type r: double
    :return: tuple(6)

    """
    print('###########################')
    print('calculating sphere inertia')
    print('###########################')

    i = 0.4 * mass * r**2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateCapsuleInertia(mass, r, h):
    """
    Returns upper diagonal of inertia tensor of a capsule as tuple.

    Code adapted from http://www.gamedev.net/page/resources/_/technical/math-and-physics/capsule-inertia-tensor-r3856

    :param mass: The capsule's mass.
    :type mass: float.
    :param r: The capsule's radius.
    :param h: float.
    :return: tuple(6).
    """
    print('###########################')
    print('calculating capsule inertia')
    print('###########################')

    cylinder_volume = math.pi * r**2 * h
    hemisphere_volume = ((4/3) * math.pi * r**3) / 2
    volume = cylinder_volume + 2*hemisphere_volume

    cylinder_mass = mass / (volume / cylinder_volume)
    hemisphere_mass = mass / (volume / hemisphere_volume)
    cylinder_height = h - 2*r

    temp0 = hemisphere_mass * 2.0 * r**2 / 5.0
    temp1 = cylinder_height * 0.5
    temp2 = temp0 + hemisphere_mass * (temp1**2 + 0.375 * cylinder_height * r)

    ixx = (r**2 * cylinder_mass / 2.0) / 2.0 + cylinder_mass * cylinder_height**2 / 12.0 + temp2 * 2.0
    ixy = 0
    ixz = 0
    iyy = ixx
    iyz = 0
    izz = r**2 * cylinder_mass / 2.0 + temp0 * 2.0

    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateEllipsoidInertia(mass, size):
    """Returns upper diagonal of inertia tensor of an ellipsoid as tuple.

    :param mass: The ellipsoids mass.
    :type mass: double
    :param size: The ellipsoids size.
    :type r: double
    :return: tuple(6)

    """
    print('###########################')
    print('calculating ellipsoid inertia')
    print('###########################')

    i = mass / 5
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return (ixx, ixy, ixz, iyy, iyz, izz,)


def calculateMeshInertia(data, mass):
    """
    Calculate the inertia tensor of arbitrary mesh objects.

    Implemented after the general idea of 'Finding the Inertia Tensor of a 3D Solid Body,
    Simply and Quickly' (2004) by Jonathan Blow (1)
    with formulas for tetrahedron inertia from 'Explicit Exact Formulas for the 3-D Tetrahedron
    Inertia Tensor in Terms of its Vertex Coordinates' (2004) by F. Tonon. (2)

    Links: (1) http://number-none.com/blow/inertia/body_i.html
           (2) http://docsdrive.com/pdfs/sciencepublications/jmssp/2005/8-11.pdf

    :param data: The mesh object's data.
    :type data: bpy.types.BlendData.
    :param mass: The object's mass.
    :type mass: float.
    :return: tuple(6)
    """

    print('###########################')
    print('calculating mesh inertia')
    print('###########################')

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
        normal_angle = ref_tri_vector.angle(tri_normal, 90)
        if normal_angle > 90:
            sign = -1
        elif normal_angle == 90:
            sign = 0
        else:
            sign = 1

        J = mathutils.Matrix(((verts[0][0], verts[0][1], verts[0][2], 1),
                              (verts[1][0], verts[1][1], verts[1][2], 1),
                              (verts[2][0], verts[2][1], verts[2][2], 1),
                              (origin[0], origin[1], origin[2], 1)))

        abs_det_J = J.determinant()

        volume = sign * abs_det_J / 6

        centre_of_mass = mathutils.Vector(((verts[0][0] + verts[1][0] + verts[2][0] + origin[0]) / 4,
                                           (verts[0][1] + verts[1][1] + verts[2][1] + origin[1]) / 4,
                                           (verts[0][2] + verts[1][2] + verts[2][2] + origin[2]) / 4))

        tetrahedra.append({'sign': sign, 'abs(det(J))': abs_det_J, 'J': J, 'centre_of_mass': centre_of_mass})
        mesh_volume += volume

    d = mass / mesh_volume
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

        a = sign * d * abs_det_J * (y1**2 + y1*y2 + y2**2 + y1*y3 + y2*y3 + y3**2
            + y1*y4 + y2*y4 + y3*y4 + y4**2 + z1**2 + z1*z2 + z2**2 + z1*z3
            + z2*z3 + z3**2 + z1*z4 + z2*z4 + z3*z4 + z4**2) / 60

        b = sign * d * abs_det_J * (x1**2 + x1*x2 + x2**2 + x1*x3 + x2*x3 + x3**2
            + x1*x4 + x2*x4 + x3*x4 + x4**2 + z1**2 + z1*z2 + z2**2 + z1*z3
            + z2*z3 + z3**2 + z1*z4 + z2*z4 + z3*z4 + z4**2) / 60

        c = sign * d * abs_det_J * (x1**2 + x1*x2 + x2**2 + x1*x3 + x2*x3 + x3**2
            + x1*x4 + x2*x4 + x3*x4 + x4**2 + y1**2 + y1*y2 + y2**2 + y1*y3
            + y2*y3 + y3**2 + y1*y4 + y2*y4 + y3*y4 + y4**2) / 60

        a_bar = sign * d * abs_det_J * (2*y1*z1 + y2*z1 + y3*z1 + y4*z1 + y1*z2
                + 2*y2*z2 + y3*z2 + y4*z2 + y1*z3 + y2*z3 + 2*y3*z3
                + y4*z3 + y1*z4 + y2*z4 + y3*z4 + 2*y4*z4) / 120

        b_bar = sign * d * abs_det_J * (2*x1*z1 + x2*z1 + x3*z1 + x4*z1 + x1*z2
                + 2*x2*z2 + x3*z2 + x4*z2 + x1*z3 + x2*z3 + 2*x3*z3
                + x4*z3 + x1*z4 + x2*z4 + x3*z4 + 2*x4*z4) / 120

        c_bar = sign * d * abs_det_J * (2*x1*y1 + x2*y1 + x3*y1 + x4*y1 + x1*y2
                + 2*x2*y2 + x3*y2 + x4*y2 + x1*y3 + x2*y3 + 2*x3*y3
                + x4*y3 + x1*y4 + x2*y4 + x3*y4 + 2*x4*y4) / 120

        i += inertiaListToMatrix([a, -b_bar, -c_bar, b, -a_bar, c])

    return i[0][0], i[0][1], i[0][2], i[1][1], i[1][2], i[2][2]


def inertiaListToMatrix(il):
    """Takes a tuple or list representing the upper diagonal of a 3x3 inertia tensor and returns the full tensor.

    :param il: The upper diagonal of a 3x3 inertia tensor.
    :type il: tuple(6) or list[6]
    :return: mathutil.Matrix

    """
    if type(il) == mathutils.Matrix:
        return il
    inertia = [[il[0], il[1], il[2]],
               [0.0, il[3], il[4]],
               [0.0, 0.0, il[5]]]
    return mathutils.Matrix(inertia)


def inertiaMatrixToList(im):
    """Takes a full 3x3 inertia tensor and returns a tuple representing the upper diagonal.

    :param im: The inertia tensor matrix.
    :type im: mathutil.Matrix
    :return: tuple(6)
    """
    return (im[0][0], im[0][1], im[0][2], im[1][1], im[1][2], im[2][2],)


def getInertiaRelevantObjects(link):
    """Returns a list of visual and collision objects of a link.
    If name-pairs of visual and collision objects are detected,
    the one with the latest change-date is used. If this is not clear,
    visual objects are omitted in favor of collision objects.

    :param link: The link you want to gather the inertia relevant objects for.
    :type link: bpy_types.Object
    :return: list

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

    *mass*: double
    *com*: mathutils:Vector(3)
    *inertia*: mathutils:Matrix(3)

    :param inertials: The alist of objects relevant for the inertia of a link.
    :type inertials: list
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
    :type obj: bpy_types.Object
    :return: bpy_types.Object -- the newly created inertia.

    """
    if obj.phobostype == 'link':
        parent = obj
        size = (0.04, 0.04, 0.04)
    else:
        parent = obj.parent
        size = (0.02, 0.02, 0.02)
    rotation = obj.matrix_world.to_euler()
    center = obj.matrix_world.to_translation()
    inertial = blenderUtils.createPrimitive('inertial_' + namingUtils.getObjectName(obj, phobostype="link"), 'box', size,
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
    """Creates inertial representations for visual and collision objects in link.

    :param link: The link you want to create the inertial for.
    :type link: bpy_types.Object
    :param empty: If set to True the new inertial object will contain no information.
    :type empty: bool
    :param preserve_children: If set to False already existent inertial objects will be deleted.
    :type preserve_children: bool

    """
    #
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
                    if geometry['type'] == 'mesh':
                        selectionUtils.selectObjects([obj])
                        bpy.context.scene.objects.active = obj
                        inert = calculateMeshInertia(obj.data, mass)
                        #print('mesh:', inert)
                        #print('ellipsoid:', calculateEllipsoidInertia(mass, geometry['size']))
                        #print('box:', calculateBoxInertia(mass, geometry['size']))
                    else:
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
    """Combines center the COG of a list of bodies given with their masses and their centers of gravity

    :param objects: The list of objects you want to combine the COG.
    :type objects: list containing phobos dicts

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
        if 'rot' in obj and not obj['rot'] == mathutils.Matrix.Identity(3): #if object is rotated in local space
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

