#!/usr/bin/python

"""
Copyright 2015, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File sensors.py

Created on 9 Apr 2015

@author: Stefan Rahms
"""

import bpy
import mathutils

from bpy.props import StringProperty, EnumProperty


def register():
    print("Registering sensors...")


def unregister():
    print("Unregistering sensors...")



def addLight(light_dict):
    dims = ['x', 'y', 'z']
    position = light_dict['position']
    direction = mathutils.Vector([light_dict['direction'][dim] for dim in dims])
    rotation = mathutils.Vector((1, 0, 0)).rotation_difference(direction).to_euler()

    bpy.ops.object.add(type='LAMP',
                       location=[position[dim] for dim in dims],
                       rotation=[rotation.x, rotation.y, rotation.z])
    light = bpy.context.object
    light.name = light_dict['name']
    light_data = light.data
    colour_data = light_dict['color']['diffuse']
    light_data.color = [colour_data[v] for v in ['r', 'g', 'b']]
    if light_dict['type'] == 'spotlight':
        light_data.type = 'SPOT'
    elif light_dict['type'] == 'omnilight':
        light_data.type = 'POINT'
    return light


#class AddLightOperator(bpy.types.Operator):
#    bl_idname = "object.phobos_add_light"
#    bl_label = "Add a light"
#    bl_options = {'REGISTER', 'UNDO'}
#
#    light_name = StringProperty(
#        name='light_name',
#        default='new_light',
#        description='name of the light'
#    )
#
#    light_type = EnumProperty(
#        name='light_type',
#        default='omnilight',
#        items=('spotlight', 'omnilight'),
#        description='type of the sensor'
#    )#
#
#    def draw(self, context):
#        layout = self.layout
#        layout.prop(self, "light_name", text="name of the light")
#        layout.prop(self, "light_type", text="light type")
#
#    def execute(self, context):
#        position = bpy.context.scene.cursor_location
#        light_dict = {'name': self.light_name,
#                      'type': self.light_type,
#                      'position': {'x': position[0],
#                                   'y': position[1],
#                                   'z': position[2]},
#                      'direction': {'x': 1, 'y': 0, 'z': 0},
#                      'color': {'diffuse': {'r': 1, 'g': 1, 'b': 1}}}
#        addLight(light_dict)
#        return {'FINISHED'}
