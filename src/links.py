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

File links.py

Created on 14 Apr 2014
"""

import bpy
import math
import phobos.defs as defs
import phobos.utils.naming as namingUtils
import phobos.utils.blender as blenderUtils
import phobos.utils.selection as selectionUtils


def register():
    """This function is called when this module is registered to blender.

    """
    print("Registering links...")


def unregister():
    """This function is called when this module is unregistered from blender.

    """
    print("Unregistering links...")


def createLink(scale, position=None, orientation=None, name=''):
    """Creates an empty link (bone) at the current 3D cursor position.

    :param scale: This is the scale you want to apply to the new link.
    :type scale: Float array with 3 elements.
    :param position: This specifies the position of the newly created link. When not given its (0.0,0.0,0.0)
    :type position: Float array with 3 elements.
    :param orientation: This specifies the rotation of the newly created link. When not given its (0.0,0.0,0.0)
    :type orientation:Float array with 3 elements.
    :param name: This sets the name for the new link. When not given the link is nameless.
    :type name: str
    :return: bpy_types.Object

    """
    blenderUtils.toggleLayer(defs.layerTypes['link'], True)
    if position is None and orientation is None:
        bpy.ops.object.armature_add(layers=blenderUtils.defLayers([0]))
    elif position is None:
        bpy.ops.object.armature_add(rotation=orientation, layers=blenderUtils.defLayers([0]))
    elif orientation is None:
        bpy.ops.object.armature_add(location=position, layers=blenderUtils.defLayers([0]))
    else:
        bpy.ops.object.armature_add(location=position, rotation=orientation, layers=blenderUtils.defLayers([0]))
    link = bpy.context.active_object
    link.scale = [scale, scale, scale]
    bpy.ops.object.transform_apply(scale=True)
    if name:
        link.name = name
    link.phobostype = 'link'
    return link


def deriveLinkfromObject(obj, scale=0.2, parenting=True, parentobjects=False, namepartindices=[], separator='_', prefix='link'):
    """Derives a link from an object that defines a joint through its position, orientation and parent-child relationships.

    :param obj: The object you want to derive your link from.
    :type obj: bpy_types.Object
    :param scale: The scale you want to apply to the link.
    :type scale: float
    :param parenting: Whether you want to automate the parenting of the new link or not.
    :type parenting: bool.
    :param parentobjects: Whether you want to parent all the objects to the new link or not.
    :type parentobjects: bool.
    :param namepartindices: Parts of the objects name you want to reuse in the links name.
    :type namepartindices: list with two elements.
    :param separator: The separator you want to use to separate the links name with. Its '_' per default
    :type separator: str
    :param prefix: The prefix you want to use for the new links name. Its 'link' per default.
    :type prefix: str

    """
    print('Deriving link from', namingUtils.getObjectName(obj))
    nameparts = namingUtils.getObjectName(obj).split('_')
    rotation = obj.matrix_world.to_euler()
    if 'invertAxis' in obj and obj['invertAxis'] == 1:
        rotation.x += math.pi if rotation.x < 0 else -math.pi
    tmpname = namingUtils.getObjectName(obj)
    if namepartindices:
        try:
            tmpname = separator.join([nameparts[p] for p in namepartindices])
        except IndexError:
            print('Wrong name segment indices given for obj', namingUtils.getObjectName(obj))
    if prefix != '':
        tmpname = prefix + separator + tmpname
    if tmpname == namingUtils.getObjectName(obj):
        obj.name += '*'
    link = createLink(scale, obj.matrix_world.to_translation(), obj.matrix_world.to_euler(), tmpname)
    if parenting:
        if obj.parent:
            selectionUtils.selectObjects([link, obj.parent], True, 1)
            if obj.parent.phobostype == 'link':
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            else:
                bpy.ops.object.parent_set(type='OBJECT')
        children = selectionUtils.getImmediateChildren(obj)
        if parentobjects:
            children.append(obj)
        for child in children:
            selectionUtils.selectObjects([child], True, 0)
            bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
            selectionUtils.selectObjects([child, link], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')





# TODO: evaluate the parenting method below
#def deriveLinkFromVisual():
#    """Derives a link from a currently-selected, visual object, both unparented or parented to the parent joint."""
#    visual = bpy.scene.active_object
#    link = bpy.ops.object.empty_add(type='ARROWS')
#    link.matrix_world = visual.matrix_world
#    if obj.parent:
#        link.parent = visual.parent
#    obj.parent = link
#    return link
