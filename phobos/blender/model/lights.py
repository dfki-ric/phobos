#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import bpy

from ..io import blender2phobos
from ..phoboslog import log
from ..utils import editing as eUtils
from ..utils import selection as sUtils


def deriveLight(light, outpath):
    """This function handles a light entity in a scene to export it

    Args:
      entity(bpy.types.Object): The lights root object.
      outpath(str): The path to export to. Not used for light entity
      savetosubfolder(bool): If True data will be exported into subfolders. Not used for light entity
      light: 

    Returns:
      : dict - An entry for the scenes entitiesList

    """
    log("Exporting " + light["entity/name"] + " as a light entity", "INFO")
    entitypose = blender2phobos.deriveObjectPose(light)
    lightobj = sUtils.getImmediateChildren(light)[0]
    color = lightobj.data.color
    entity = {
        "name": light["entity/name"],
        "type": "light",
        "light_type": "spotlight" if lightobj.data.type == "SPOT" else "omnilight",
        "anchor": light["anchor"] if "anchor" in light else "none",
        "color": {
            "diffuse": [color.r, color.g, color.b],
            "use_specular": lightobj.data.use_specular,  # only specular information currently available
        },
        "position": entitypose["translation"],
        "rotation": entitypose["rotation_quaternion"],
    }
    if entity["light_type"] == "spotlight":
        entity["angle"] = lightobj.data.spot_size
    return entity


def addLight(light_dict):
    """

    Args:
      light_dict: 

    Returns:

    """
    # DOCU add some docstring

    if light_dict['type'] == 'spotlight':
        light_type = 'SPOT'
    elif light_dict['type'] == 'omnilight':
        light_type = 'POINT'

    position = light_dict['pose']['translation']
    rotation = light_dict['pose']['rotation_euler']

    bpy.ops.object.lamp_add(type=light_type, location=position, rotation=rotation)
    light = bpy.context.active_object
    if 'parent' in light_dict:
        eUtils.parentObjectsTo(light, bpy.data.objects[light_dict['parent']])

    light_data = light.data
    light.name = light_dict['name']

    colour_vals = ['r', 'g', 'b']
    colour_data = light_dict['color']['diffuse']
    light_data.color = [colour_data[v] for v in colour_vals]
    for v in colour_vals:
        if light_dict['color']['specular'][v] > 0:
            light_data.use_specular = True
            break

    if type == 'SPOT':
        light_data.spot_size = light_dict['angle']

    # TODO delete me?
    # if light_dict['attenuation']['constant'] > 0:
    light_data.energy = light_dict['attenuation']['constant']
    falloff = 'CONSTANT'
    if light_dict['attenuation']['linear'] > 0:
        light_data.linear_attenuation = light_dict['attenuation']['linear']
        falloff = 'INVERSE_LINEAR'
    if light_dict['attenuation']['quadratic'] > 0:
        light_data.quadratic_attenuation = light_dict['attenuation']['quadratic']
        if falloff == 'INVERSE_LINEAR':
            falloff = 'LINEAR_QUADRATIC_WEIGHTED'
        else:
            falloff = 'INVERSE_SQUARE'
    light_data.falloff_type = falloff

    light.phobostype = 'light'
    light['light/exponent'] = light_dict['exponent']
    light.phobostype = 'light'
    light['light/directional'] = light_dict['directional']
    return light


# TODO move operator to other operators and give it a dev branch
# class AddLightOperator(bpy.types.Operator):
#    bl_idname = "phobos.add_light"
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
