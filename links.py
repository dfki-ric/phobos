#!/usr/bin/python

"""
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

@author: Kai von Szadkowski
"""

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty
import math
import mathutils
import warnings

def register():
    print("Registering links...")


def unregister():
    print("Unregistering links...")

def createLink(joints):
    '''Creates an empty link (bone) at the current 3D cursor position.'''
    link = bpy.ops.object.empty_add(type='ARROWS', location = bpy.context.scene.cursor_location)
    link.MARStype = 'link'
    return link

def deriveLinksfromJoints():
    joints = bpy.context.selected_objects
    for j in joints:
        rotation = j.matrix_world.to_euler()
        if 'invertAxis' in j and j['invertAxis'] == 1:
            rotation.x += math.pi if rotation.x < 0 else -math.pi
        bpy.ops.object.armature_add(location=j.matrix_world.to_translation(),
                                    rotation=j.matrix_world.to_euler())
        link = bpy.context.active_object
        link.scale = [0.1, 0.1, 0.1]
        link['turned'] = True

def deriveLinkFromVisual():
    '''Derives a link from a currently-selected, visual object, both unparented or parented to the parent joint.'''
    visual = bpy.scene.active_object
    link = bpy.ops.object.empty_add(type='ARROWS')
    link.matrix_world = visual.matrix_world
    if visual.parent:
        link.parent = visual.parent
    visual.parent = link
    return link
