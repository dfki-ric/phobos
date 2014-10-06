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
from bpy.types import Operator
from bpy.props import StringProperty, FloatProperty
from . import materials
from . import defs
from . import utility


def register():
    print("Registering sensors...")

def unregister():
    print("Unregistering sensors...")


class AddSensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.phobos_add_sensor"
    bl_label = "Add/Update a sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = StringProperty(
        name = "sensor_type",
        default = "",
        description = "type of the sensor to be created")

    sensor_scale = FloatProperty(
        name = "sensor_scale",
        default = 0.05,
        description = "scale of the sensor visualization")

    sensor_name = StringProperty(
        name = "sensor_name",
        default = 'new_sensor',
        description = "name of the sensor")

    def execute(self, context):
        location = bpy.context.scene.cursor_location
        if self.sensor_type in defs.sensortypes:
            if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
                objects = []
                sensors = []
                for obj in bpy.context.selected_objects:
                    if obj.phobostype == "sensor":
                        sensors.append(obj)
                        self.sensor_type = obj["sensor/type"]
                    else:
                        objects.append(obj)
                if len(sensors) <= 0:
                    sense = utility.createPrimitive(self.sensor_type, "sphere", self.sensor_scale, defs.layerTypes["sensor"], "sensor", location)
                    sense.phobostype = "sensor"
                    sense.name = self.sensor_name if self.sensor_name != '' else 'new_' + self.sensor_type
                    sense["sensor/type"] = self.sensor_type
                    sensors.append(sense)
                    sense.parent = utility.getRoot(objects[0])
                for sensor in sensors:
                    if "Node" in sensor["sensor/type"]:
                        sensor['nodes'] = [obj for obj in objects if obj.phobostype == 'collision']
                    elif "Joint" in sensor["sensor/type"] or "Motor" in sensor["sensor/type"]:
                        sensor['joints'] = [obj for obj in objects if obj.phobostype == 'link']
            else: #visual sensor
                if self.sensor_type in ["RaySensor", "RotatingRaySensor", "ScanningSonar"]:
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "CameraSensor":
                    bpy.ops.object.add(type='CAMERA', location=bpy.context.scene.cursor_location)
                    sensor = bpy.context.active_object
                    print("Added nodes to new " + self.sensor_type)
            for prop in defs.sensorProperties[self.sensor_type]:
                sensor[prop] = defs.sensorProperties[self.sensor_type][prop]
        else:
            print("Error: Sensor could not be created: unknown sensor type.")
        return {'FINISHED'}


class AddLegacySensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.phobos_add_legacy_sensor"
    bl_label = "Add/Update a sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = StringProperty(
        name = "sensor_type",
        default = "",
        description = "type of the sensor to be created")

    sensor_scale = FloatProperty(
        name = "sensor_scale",
        default = 0.05,
        description = "scale of the sensor visualization")

    def execute(self, context):
        location = bpy.context.scene.cursor_location
        if self.sensor_type in defs.sensortypes:
            if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
                objects = []
                sensors = []
                for obj in bpy.context.selected_objects:
                    if obj.phobostype == "sensor":
                        sensors.append(obj)
                        self.sensor_type = obj["sensor/type"]
                    else:
                        objects.append(obj)
                if len(sensors) <= 0:
                    utility.createPrimitive(self.sensor_type, "sphere", self.sensor_scale, defs.layerTypes["sensor"], "sensor", location)
                    sense = bpy.context.scene.objects.active
                    sense.phobostype = "sensor"
                    sense.name = self.sensor_type
                    sense["sensor/type"] = self.sensor_type
                    sensors.append(sense)
                for sensor in sensors:
                    for key in sensor.keys():
                        if key.find("index") >= 0:
                            del sensor[key]
                            print("Deleting " + key + " in " + sensor.name)
                    i = 1
                    if "Node" in sensor["sensor/type"]:
                        for obj in objects:
                            if obj.phobostype == "collision":
                                sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                                i += 1
                        print("Added nodes to new " + self.sensor_type)
                    elif "Joint" in sensor["sensor/type"] or "Motor" in sensor["sensor/type"]:
                        for obj in objects:
                            if obj.phobostype == "link":
                                sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                                i += 1
                        print("Added nodes to new " + self.sensor_type)
                    if len(objects) > 0:
                        sensor.parent = utility.getRoot(objects[0])
            else: #visual sensor
                if self.sensor_type == "RaySensor":
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "CameraSensor":
                    bpy.ops.object.add(type='CAMERA', location=bpy.context.scene.cursor_location)
                    sensor = bpy.context.active_object
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "ScanningSonar":
                    print("Added nodes to new " + self.sensor_type)
            # add the pre-defined sensor properties
            for prop in defs.sensorProperties[self.sensor_type]:
                sensor[prop] = defs.sensorProperties[self.sensor_type][prop]
        else:
            print("Sensor could not be created: unknown sensor type.")
        return {'FINISHED'}
