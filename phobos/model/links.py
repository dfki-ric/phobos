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
import math
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
from phobos.logging import log


def createLink(self, link):
        """This function creates the blender representation of a given link

        :param link: The link you want to create a representation of.
        :type link: dict
        :return: bpy_types.Object -- the newly created blender link object.

        """
        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
        #create base object ( =armature)
        bpy.ops.object.select_all(action='DESELECT')
        #bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=bUtils.defLayers(0))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink["link/name"] = link['name']
        newlink.name = self.praefixNames(link['name'], "link")
        #newlink.location = (0.0, 0.0, 0.0)
        newlink.location = link['pose']['translation']
        newlink.scale = (0.3, 0.3, 0.3) #TODO: make this depend on the largest visual or collision object
        bpy.ops.object.transform_apply(scale=True)
        newlink.phobostype = 'link'
        if newlink.name != self.praefixNames(link['name'], "link"):
            log("Warning, name conflict!")
        # place inertial
        if 'inertial' in link:
            self.createInertial(link['name'], link['inertial'])
        # place visual
        if 'visual' in link:
            for v in link['visual']:
                visual = link['visual'][v]
                if 'geometry' in visual:
                    self.createGeometry(visual, 'visual')
        # place collision
        if 'collision' in link:
            for c in link['collision']:
                collision = link['collision'][c]
                if 'geometry' in collision:
                    self.createGeometry(collision, 'collision')
        for prop in link:
            if prop.startswith('$'):
                for tag in link[prop]:
                    newlink['link/'+prop[1:]+'/'+tag] = link[prop][tag]
        return newlink


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
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    if position is None and orientation is None:
        bpy.ops.object.armature_add(layers=bUtils.defLayers([0]))
    elif position is None:
        bpy.ops.object.armature_add(rotation=orientation, layers=bUtils.defLayers([0]))
    elif orientation is None:
        bpy.ops.object.armature_add(location=position, layers=bUtils.defLayers([0]))
    else:
        bpy.ops.object.armature_add(location=position, rotation=orientation, layers=bUtils.defLayers([0]))
    link = bpy.context.active_object
    link.scale = [scale, scale, scale]
    bpy.ops.object.transform_apply(scale=True)
    if name:
        link.name = name
    link.phobostype = 'link'
    return link


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
