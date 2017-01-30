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
import math
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.model.inertia as inertiamodel
import phobos.model.geometries as geometrymodel
from phobos.phoboslog import log


def getGeometricElements(link):
    visuals = []
    collisions = []
    if 'visual' in link:
        visuals = [link['visual'][v] for v in link['visual']]
    if 'collision' in link:
        collisions = [link['collision'][v] for v in link['collision']]
    return visuals + collisions


def createLink(link):
    """Creates the blender representation of a given link and its parent joint.

    :param link: The link you want to create a representation of.
    :type link: dict
    :return: bpy_types.Object -- the newly created blender link object.

    """
    # create armature/bone
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.armature_add(layers=bUtils.defLayers([defs.layerTypes['link']]))
    newlink = bpy.context.active_object
    newlink.phobostype = 'link'
    newlink.name = link['name']
    # FIXME: This is a hack and should be checked before creation!
    newlink["link/name"] = link['name']  # this is a backup in case an object with the link's name already exists

    # set the size of the link
    elements = getGeometricElements(link)
    scale = max((geometrymodel.getLargestDimension(element['geometry']) for element in elements)) if elements else 0.2
    newlink.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(scale=True)

    # create inertial
    if 'inertial' in link:
        inertiamodel.createInertialFromDictionary(link['name'], link['inertial'])

    # create visual elements
    if 'visual' in link:
        for v in link['visual']:
            visual = link['visual'][v]
            geometrymodel.createGeometry(visual, 'visual')

    # create collision elements
    if 'collision' in link:
        for c in link['collision']:
            collision = link['collision'][c]
            geometrymodel.createGeometry(collision, 'collision')

    # add custom properties
    for prop in link:
        if prop.startswith('$'):
            for tag in link[prop]:
                newlink['link/'+prop[1:]+'/'+tag] = link[prop][tag]

    return newlink


def deriveLinkfromObject(obj, scale=0.2, parenting=True, parentobjects=False, namepartindices=[], separator='_', prefix='link'):
    """Derives a link from an object that defines a joint through its position, orientation and parent-child relationships.

    :param obj: The object to derive a link from.
    :type obj: bpy_types.Object
    :param scale: The scale to apply to the link.
    :type scale: float
    :param parenting: Whether to automate the parenting of the new link or not.
    :type parenting: bool.
    :param parentobjects: Whether to parent all the objects to the new link or not.
    :type parentobjects: bool.
    :param namepartindices: Parts of the objects name you want to reuse in the links name.
    :type namepartindices: list with two elements.
    :param separator: The separator to use to separate the links name with. Its '_' per default
    :type separator: str
    :param prefix: The prefix to use for the new links name. Its 'link' per default.
    :type prefix: str

    """
    print('Deriving link from', nUtils.getObjectName(obj))
    nameparts = nUtils.getObjectName(obj).split('_')
    rotation = obj.matrix_world.to_euler()
    if 'invertAxis' in obj and obj['invertAxis'] == 1:
        rotation.x += math.pi if rotation.x < 0 else -math.pi
    tmpname = nUtils.getObjectName(obj)
    if namepartindices:
        try:
            tmpname = separator.join([nameparts[p] for p in namepartindices])
        except IndexError:
            print('Wrong name segment indices given for obj', nUtils.getObjectName(obj))
    if prefix != '':
        tmpname = prefix + separator + tmpname
    if tmpname == nUtils.getObjectName(obj):
        obj.name += '*'
    link = createLink(scale, obj.matrix_world.to_translation(), obj.matrix_world.to_euler(), tmpname)
    if parenting and obj.parent:
        if obj.parent:
            sUtils.selectObjects([link, obj.parent], True, 1)
            if obj.parent.phobostype == 'link':
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            else:
                bpy.ops.object.parent_set(type='OBJECT')
        children = sUtils.getImmediateChildren(obj)
        if parentobjects:
            children.append(obj)
        for child in children:
            sUtils.selectObjects([child], True, 0)
            bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
            sUtils.selectObjects([child, link], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')


def placeChildLinks(model, parent):
    """Creates parent-child-relationship for a given parent and all existing children in Blender.

    :param parent: This is the parent link you want to set the children for.
    :type: dict

    """
    bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
    children = []
    for l in model['links']:
        if 'parent' in model['links'][l] and model['links'][l]['parent'] == parent['name']:
            children.append(model['links'][l])
    for child in children:
        # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
        parentLink = bpy.data.objects[parent['name']]
        childLink = bpy.data.objects[child['name']]
        sUtils.selectObjects([childLink, parentLink], True, 1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        # 2: move to parents origin by setting the world matrix to the parents world matrix
        childLink.matrix_world = parentLink.matrix_world        # removing this line does not seem to make a difference

        # #bpy.context.scene.objects.active = childLink
        # if 'pivot' in child:
        #     pivot = child['pivot']
        #     cursor_location = bpy.context.scene.cursor_location
        #     bpy.context.scene.cursor_location = mathutils.Vector((-pivot[0]*0.3, -pivot[1]*0.3, -pivot[2]*0.3))
        #     bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        #     bpy.context.scene.cursor_location = cursor_location

        # 3: apply local transform as saved in model (changes matrix_local)
        location = mathutils.Matrix.Translation(child['pose']['translation'])
        rotation = mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        transform_matrix = location * rotation
        childLink.matrix_local = transform_matrix
        # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
        # 5: take care of the rest of the tree
        placeChildLinks(model, child)


def placeLinkSubelements(link):
    """Finds all subelements for a given link and sets the appropriate relations.
    In this case subelements are interials, visuals and collisions.

    :param link: The parent link you want to set the subelements for
    :type link: dict

    """
    elements = getGeometricElements(link) + ([link['inertial']] if 'inertial' in link else [])
    bpy.context.scene.layers = bUtils.defLayers([defs.layerTypes[t] for t in defs.layerTypes])
    parentlink = bpy.data.objects[link['name']]
    for element in elements:
        if 'pose' in element:
            location = mathutils.Matrix.Translation(element['pose']['translation'])
            rotation = mathutils.Euler(tuple(element['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        else:
            location = mathutils.Matrix.Identity(4)
            rotation = mathutils.Matrix.Identity(4)
        try:
            obj = bpy.data.objects[element['name']]
        except KeyError:
            log('Missing link element for placement: ' + element['name'], 'ERROR', 'placeLinkSubelements')
            continue
        sUtils.selectObjects([obj, parentlink], True, 1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        obj.matrix_local = location * rotation
        try:
            obj.scale = mathutils.Vector(element['geometry']['scale'])
        except KeyError:
            pass
        # sUtils.selectObjects([element, parentlink], True, 1)
