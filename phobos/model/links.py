#!/usr/bin/python
# coding=utf-8

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

File links.py

Created on 14 Apr 2014
"""

import bpy
import mathutils
import re
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.model.inertia as inertia
import phobos.model.geometries as geometrymodel
from phobos.phoboslog import log


def getGeometricElements(link):
    """Returns all geometric elements of a link, i.e. 'visual' and 'collision' objects.
    Args:
        link(dict): definition of link

    Returns(list): lists of visual and collision object definitions

    """
    visuals = []
    collisions = []
    if 'visual' in link:
        visuals = [link['visual'][v] for v in link['visual']]
    if 'collision' in link:
        collisions = [link['collision'][v] for v in link['collision']]
    return visuals, collisions


# DOCU we should add the parameters, that can be inserted in the dictionary
def createLink(link):
    """Creates the blender representation of a given link and its parent joint.

    Args:
      link(dict): The link you want to create a representation of.

    Returns:
      bpy_types.Object -- the newly created blender link object.

    """
    # create armature/bone
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.armature_add(layers=bUtils.defLayers([defs.layerTypes['link']]))
    newlink = bpy.context.active_object

    # Move bone when adding at selected objects location
    if 'matrix' in link:
        newlink.matrix_world = link['matrix']
    newlink.phobostype = 'link'
    if link['name'] in bpy.data.objects.keys():
        log('Object with name of new link already exists: ' + link['name'], 'WARNING')
    nUtils.safelyName(newlink, link['name'])


    # set the size of the link
    visuals, collisions = getGeometricElements(link)
    if visuals or collisions:
        scale = max((geometrymodel.getLargestDimension(e['geometry'])
                     for e in visuals + collisions))
    else:
        scale = 0.2

    # use scaling factor provided by user
    if 'scale' in link:
        scale *= link['scale']
    newlink.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(scale=True)

    # add custom properties
    for prop in link:
        if prop.startswith('$'):
            for tag in link[prop]:
                newlink['link/' + prop[1:] + '/' + tag] = link[prop][tag]

    # create inertial
    if 'inertial' in link:
        inertia.createInertial(link['inertial'], newlink)

    # create geometric elements
    log("Creating visual and collision objects for link '{0}': {1}".format(
        link['name'], ', '.join([elem['name'] for elem in visuals + collisions])), 'DEBUG')
    for v in visuals:
            geometrymodel.createGeometry(v, 'visual', newlink)
    for c in collisions:
            geometrymodel.createGeometry(c, 'collision', newlink)
    return newlink


def deriveLinkfromObject(obj, scale=0.2, parent_link=True, parent_objects=False, nameformat=''):
    """Derives a link from an object using its name, transformation and parenting.

    Args:
      obj(bpy_types.Object): object to derive a link from
      scale(float, optional): scale factor for bone size (Default value = 0.2)
      parent_link(bool, optional): whether to automate the parenting of the new link or not. (Default value = True)
      parent_objects(bool, optional): whether to parent all the objects to the new link or not (Default value = False)
      nameformat(str, optional): re-formatting template for obj names (Default value = '')

    Returns:
      newly created link

    """
    log('Deriving link from ' + nUtils.getObjectName(obj), level="INFO")
    try:
        nameparts = [p for p in re.split('[^a-zA-Z]', nUtils.getObjectName(obj)) if p != '']
        linkname = nameformat.format(*nameparts)
    except IndexError:
        log('Invalid name format (indices) for naming: ' + nUtils.getObjectName(obj), 'WARNING')
        linkname = 'link_' + nUtils.getObjectName(obj)
    link = createLink({'scale': scale, 'name': linkname, 'matrix': obj.matrix_world})

    # parent link to object's parent
    if parent_link:
        if obj.parent:
            sUtils.selectObjects([link, obj.parent], True, 1)
            if obj.parent.phobostype == 'link':
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            else:
                bpy.ops.object.parent_set(type='OBJECT')
    # parent children of object to link
    if parent_objects:
        children = [obj] + sUtils.getImmediateChildren(obj)
        sUtils.selectObjects(children, True, 0)
        bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
        sUtils.selectObjects([link] + children, True, 0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
    return link


def placeChildLinks(model, parent):
    """Creates parent-child-relationship for a given parent and all existing children in Blender.

    Args:
      parent(dict): parent link you want to set the children for.
      model(dict):

    Returns:

    """
    bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
    for c in parent['children']:
        child = model['links'][c]
        # apply transform as saved in model
        location = mathutils.Matrix.Translation(child['pose']['translation'])
        rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        log('Placing link {0}'.format(child['name']), 'DEBUG')
        transform_matrix = location * rotation
        log("Joint transform: {0}".format(transform_matrix), 'DEBUG')
        child['object'].matrix_local = transform_matrix

        # traverse the tree
        placeChildLinks(model, child)
