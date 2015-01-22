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
from bpy.types import Operator
from bpy.props import FloatProperty, BoolProperty, EnumProperty, StringProperty
import math
import warnings
from . import utility
from . import defs


def register():
    """
    This function registers this module.
    At the moment it does nothing.

    :return: Nothing

    """
    print("Registering links...")


def unregister():
    """
    This function unregisters this module.
    At the moment it does nothing.

    :return: Nothing

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
    :type name: string.
    :return: blender object

    """
    utility.toggleLayer(defs.layerTypes['link'], True)
    if position is None and orientation is None:
        bpy.ops.object.armature_add(layers=utility.defLayers([0]))
    elif position is None:
        bpy.ops.object.armature_add(rotation=orientation, layers=utility.defLayers([0]))
    elif orientation is None:
        bpy.ops.object.armature_add(location=position, layers=utility.defLayers([0]))
    else:
        bpy.ops.object.armature_add(location=position, rotation=orientation, layers=utility.defLayers([0]))
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
    :type obj: Blender object.
    :param scale: The scale you want to apply to the link.
    :type scale: float.
    :param parenting: Whether you want to automate the parenting of the new link or not.
    :type parenting: bool.
    :param parentobjects: Whether you want to parent all the objects to the new link or not.
    :type parentobjects: bool.
    :param namepartindices: Parts of the objects name you want to reuse in the links name.
    :type namepartindices: list with two elements.
    :param separator: The separator you want to use to separate the links name with. Its '_' per default
    :type separator: string.
    :param prefix: The prefix you want to use for the new links name. Its 'link' per default.
    :type prefix: string.
    :return: Nothing.

    """
    print('Deriving link from', obj.name)
    nameparts = obj.name.split('_')
    rotation = obj.matrix_world.to_euler()
    if 'invertAxis' in obj and obj['invertAxis'] == 1:
        rotation.x += math.pi if rotation.x < 0 else -math.pi
    tmpname = obj.name
    if namepartindices:
        try:
            tmpname = separator.join([nameparts[p] for p in namepartindices])
        except IndexError:
            print('Wrong name segment indices given for obj', obj.name)
    if prefix != '':
        tmpname = prefix + separator + tmpname
    if tmpname == obj.name:
        obj.name += '*'
    link = createLink(scale, obj.matrix_world.to_translation(), obj.matrix_world.to_euler(), tmpname)
    if parenting:
        if obj.parent:
            utility.selectObjects([link, obj.parent], True, 1)
            if obj.parent.phobostype == 'link':
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            else:
                bpy.ops.object.parent_set(type='OBJECT')
        children = utility.getImmediateChildren(obj)
        if parentobjects:
            children.append(obj)
        for child in children:
            utility.selectObjects([child], True, 0)
            bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
            utility.selectObjects([child, link], True, 1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')


class CreateLinkOperator(Operator):
    """Create Link Operator

    """
    bl_idname = "object.phobos_create_link"
    bl_label = "Create link(s), optionally based on existing objects"
    bl_options = {'REGISTER', 'UNDO'}

    type = EnumProperty(
        items=(('3D cursor',)*3,
               ('selected objects',)*3),
        default='selected objects',
        name='location',
        description='Where to create new link(s)?'
    )

    size = FloatProperty(
        name="visual link size",
        default=0.2,
        description="size of the created link"
    )

    parenting = BoolProperty(
        name='parenting',
        default=False,
        description='parent associated objects to created links?'
    )

    parentobject = BoolProperty(
        name='parent object(s)',
        default=False,
        description='parent objects to newly created links?'
    )

    namepartindices = StringProperty(
        name="name segment indices",
        description="allows reusing parts of objects' names, specified as e.g. '2 3'",
        default=''
    )

    separator = StringProperty(
        name="separator",
        description="seperator to split object names with, e.g. '_'",
        default='_'
    )

    prefix = StringProperty(
        name="prefix",
        description="prefix to put before names, e.g. 'link'",
        default='link'
    )

    def execute(self, context):
        """This function executes the operator and creates a link.

        :param context: The blender context this operator works with.
        :type context: Blender context.
        :return: Blender result.
        """
        if self.type == '3D cursor':
            createLink(self.size)
        else:
            for obj in bpy.context.selected_objects:
                tmpnamepartindices = [int(p) for p in self.namepartindices.split()]
                deriveLinkfromObject(obj, scale=self.size, parenting=self.parenting, parentobjects=self.parentobject,
                                     namepartindices=tmpnamepartindices, separator=self.separator,
                                     prefix=self.prefix)
        return {'FINISHED'}


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
