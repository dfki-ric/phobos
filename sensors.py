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

File sensors.py

Created on 6 Jan 2014

@author: Kai von Szadkowski
"""


import bpy
import mathutils
from bpy.types import Operator
from bpy.props import StringProperty, FloatProperty, EnumProperty, IntProperty, BoolProperty, FloatVectorProperty
from . import materials
from . import defs
from . import utility
from . import links


def register():
    print("Registering sensors...")


def unregister():
    print("Unregistering sensors...")


def createSensor(sensor, reference, origin):
    utility.toggleLayer(defs.layerTypes['sensor'], value=True)
    # create sensor object
    if 'Camera' in sensor['type']:
        bpy.context.scene.layers[defs.layerTypes['sensor']] = True
        bpy.ops.object.add(type='CAMERA', location=origin.to_translation(),
                           rotation=origin.to_euler(),
                           layers=utility.defLayers([defs.layerTypes['sensor']]))
        newsensor = bpy.context.active_object
        if reference is not None and reference != []:
            utility.selectObjects([newsensor, bpy.data.objects[reference]], clear=True, active=1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
    elif sensor['type'] in ['RaySensor', 'RotatingRaySensor', 'ScanningSonar', 'MultiLevelLaserRangeFinder']:
        # TODO: create a proper ray sensor scanning layer disc here
        newsensor = utility.createPrimitive(sensor['name'], 'disc', (0.5, 36),
                                            defs.layerTypes['sensor'], 'phobos_laserscanner',
                                            origin.to_translation(), protation=origin.to_euler())
        if reference is not None and reference != []:
            utility.selectObjects([newsensor, bpy.data.objects[reference]], clear=True, active=1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
    elif sensor['type'] in ['Joint6DOF']:
        newsensor = utility.createPrimitive(sensor['name'], 'sphere', 0.05,
                                            defs.layerTypes['sensor'], 'phobos_sensor',
                                            origin.to_translation(), protation=origin.to_euler())
        utility.selectObjects([newsensor, reference], clear=True, active=1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
    else:  # contact, force and torque sensors (or unknown sensors)
        newsensor = utility.createPrimitive(sensor['name'], 'sphere', 0.05,
                                            defs.layerTypes['sensor'], 'phobos_sensor',
                                            origin.to_translation(), protation=origin.to_euler())
        if 'Node' in sensor['type']:
            newsensor['sensor/nodes'] = sorted([utility.getObjectName(obj) for obj in reference])
        elif 'Joint' in sensor['type'] or 'Motor' in sensor['type']:
            newsensor['sensor/joints'] = sorted([utility.getObjectName(obj) for obj in reference])
        if reference is not None and reference != []:
            utility.selectObjects([newsensor, utility.getRoot(reference[0])], clear=True, active=1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
    # set sensor properties
    newsensor.phobostype = 'sensor'
    newsensor.name = sensor['name']
    newsensor['sensor/type'] = sensor['type']
    for prop in sensor['props']:
        newsensor['sensor/'+prop] = sensor['props'][prop]

    # add custom properties
    for prop in sensor:
        if prop.startswith('$'):
            for tag in sensor[prop]:
                newsensor[prop[1:]+'/'+tag] = sensor[prop][tag]

    # throw warning if type is not known
    if sensor['type'] not in defs.sensortypes:
        print("### Warning: sensor", sensor['name'], "is of unknown/custom type.")
    utility.selectObjects([newsensor], clear=False, active=0)
    return newsensor


class AddSensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.phobos_add_sensor"
    bl_label = "Add/Update a sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = EnumProperty(
        name="sensor_type",
        default="CameraSensor",
        items=tuple([(type,)*3 for type in defs.sensortypes]),
        description="tyoe of the sensor to be created"
    )

    custom_type = StringProperty(
        name="custom_type",
        default='',
        description="type of the custom sensor to be created"
    )

    sensor_name = StringProperty(
        name="sensor_name",
        default='new_sensor',
        description="name of the sensor"
    )

    add_link = BoolProperty(name="add_link", default=True, description="add additional link as sensor mounting")

# the following is a set of all properties that exist within MARS' sensors
    # TODO: we should get rid of gui-settings such as hud and rename stuff (eg. maxDistance - maxDist)
    width = IntProperty(name='width', default=0, description='width')
    height = IntProperty(name='height', default=0, description='height')
    resolution = FloatProperty(name='resolution', default=0, description='resolution')
    horizontal_resolution = FloatProperty(name='horizontal_resolution', default=0, description='horizontal_resolution')
    opening_width = FloatProperty(name='opening_width', default=0, description='opening_width')
    opening_height = FloatProperty(name='opening_height', default=0, description='opening_height')
    maxDistance = FloatProperty(name='maxDistance', default=0, description='maxDistance')
    maxDist = FloatProperty(name='maxDist', default=0, description='maxDist')
    verticalOpeningAngle = FloatProperty(name='verticalOpeningAngle', default=0, description='verticalOpeningAngle')
    horizontalOpeningAngle = FloatProperty(name='horizontalOpeningAngle', default=0, description='horizontalOpeningAngle')
    hud_pos = IntProperty(name='hud_pos', default=0, description='hud_pos')
    hud_width = IntProperty(name='hud_width', default=0, description='hud_width')
    hud_height = IntProperty(name='hud_height', default=0, description='hud_height')
    updateRate = FloatProperty(name='updateRate', default=0, description='updateRate')
    vertical_offset = FloatProperty(name='vertical_offset', default=0, description='vertical_offset')
    horizontal_offset = FloatProperty(name='horizontal_offset', default=0, description='horizontal_offset')
    gain = FloatProperty(name='gain', default=0, description='gain')
    left_limit = FloatProperty(name='left_limit', default=0, description='left_limit')
    right_limit = FloatProperty(name='right_limit', default=0, description='right_limit')
    rttResolutionX = FloatProperty(name='rttResolutionX', default=0, description='rttResolutionX')
    rttResolutionY = FloatProperty(name='rttResolutionY', default=0, description='rttResolutionY')
    numRaysHorizontal = FloatProperty(name='numRaysHorizontal', default=0, description='numRaysHorizontal')
    numRaysVertical = FloatProperty(name='numRaysVertical', default=0, description='numRaysVertical')
    draw_rays = BoolProperty(name='draw_rays', default=False, description='draw_rays')
    depthImage = BoolProperty(name='depthImage', default=False, description='depthImage')
    show_cam = BoolProperty(name='show_cam', default=False, description='show_cam')
    only_ray = BoolProperty(name='only_ray', default=False, description='only_ray')
    ping_pong_mode = BoolProperty(name='ping_pong_mode', default=False, description='ping_pong_mode')
    bands = IntProperty(name='bands', default=0, description='bands')
    lasers = IntProperty(name='lasers', default=0, description='lasers')
    extension = FloatVectorProperty(name='extension', default=(0, 0, 0), description='extension')

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "sensor_name", text="name of the sensor")
        layout.prop(self, "sensor_type", text="sensor type")
        if self.sensor_type in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                                  'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            layout.prop(self, "add_link", "attach sensor to new link?")
        if self.sensor_type == "custom":
            layout.prop(self, "custom_type", text="enter custom type")
        else:
            for key in defs.sensorProperties[self.sensor_type]:
                layout.prop(self, key, text=key)

    def execute(self, context):
        # create a dictionary holding the sensor definition
        sensor = {'name': self.sensor_name,
                  'type': self.custom_type if self.sensor_type == 'Custom' else self.sensor_type,
                  'props': {}
                 }
        parent = context.active_object
        for key in defs.sensorProperties[self.sensor_type]:
            sensor['props'][key] = getattr(self, key)
        # type-specific settings
        if sensor['type'] in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                              'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            sensorObj = createSensor(sensor, context.active_object.name, context.active_object.matrix_world)
            if self.add_link:
                link = links.createLink(scale=0.1, position=context.active_object.matrix_world.to_translation(), name='link_'+self.sensor_name)
                utility.selectObjects([parent, link], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                utility.selectObjects([link, sensorObj], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
        elif sensor['type'] in ['Joint6DOF']:
            createSensor(sensor, context.active_object, context.active_object.matrix_world)
        elif 'Node' in sensor['type']:
            createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'collision'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        elif 'Motor' in sensor['type'] or 'Joint' in sensor['type']:
            createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'link'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        return {'FINISHED'}


# class AddLegacySensorOperator(Operator):
#     """AddSensorOperator"""
#     bl_idname = "object.phobos_add_legacy_sensor"
#     bl_label = "Add/Update a sensor"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     sensor_type = StringProperty(
#         name = "sensor_type",
#         default = "",
#         description = "type of the sensor to be created")
#
#     sensor_scale = FloatProperty(
#         name = "sensor_scale",
#         default = 0.05,
#         description = "scale of the sensor visualization")
#
#     def execute(self, context):
#         location = bpy.context.scene.cursor_location
#         if self.sensor_type in defs.sensortypes:
#             if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
#                 objects = []
#                 sensors = []
#                 for obj in bpy.context.selected_objects:
#                     if obj.phobostype == "sensor":
#                         sensors.append(obj)
#                         self.sensor_type = obj["sensor/type"]
#                     else:
#                         objects.append(obj)
#                 if len(sensors) <= 0:
#                     utility.createPrimitive(self.sensor_type, "sphere", self.sensor_scale, defs.layerTypes["sensor"], "sensor", location)
#                     sense = bpy.context.scene.objects.active
#                     sense.phobostype = "sensor"
#                     sense.name = self.sensor_type
#                     sense["sensor/type"] = self.sensor_type
#                     sensors.append(sense)
#                 for sensor in sensors:
#                     for key in sensor.keys():
#                         if key.find("index") >= 0:
#                             del sensor[key]
#                             print("Deleting " + key + " in " + sensor.name)
#                     i = 1
#                     if "Node" in sensor["sensor/type"]:
#                         for obj in objects:
#                             if obj.phobostype == "collision":
#                                 sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
#                                 i += 1
#                         print("Added nodes to new " + self.sensor_type)
#                     elif "Joint" in sensor["sensor/type"] or "Motor" in sensor["sensor/type"]:
#                         for obj in objects:
#                             if obj.phobostype == "link":
#                                 sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
#                                 i += 1
#                         print("Added nodes to new " + self.sensor_type)
#                     if len(objects) > 0:
#                         sensor.parent = utility.getRoot(objects[0])
#             else: #visual sensor
#                 if self.sensor_type == "RaySensor":
#                     print("Added nodes to new " + self.sensor_type)
#                 elif self.sensor_type == "CameraSensor":
#                     bpy.ops.object.add(type='CAMERA', location=bpy.context.scene.cursor_location)
#                     sensor = bpy.context.active_object
#                     print("Added nodes to new " + self.sensor_type)
#                 elif self.sensor_type == "ScanningSonar":
#                     print("Added nodes to new " + self.sensor_type)
#             # add the pre-defined sensor properties
#             for prop in defs.sensorProperties[self.sensor_type]:
#                 sensor[prop] = defs.sensorProperties[self.sensor_type][prop]
#         else:
#             print("Sensor could not be created: unknown sensor type.")
#         return {'FINISHED'}
