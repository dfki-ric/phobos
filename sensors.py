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
from bpy.props import StringProperty, FloatProperty, EnumProperty
from . import materials
from . import defs
from . import utility


def register():
    print("Registering sensors...")


def unregister():
    print("Unregistering sensors...")


def createSensor(sensor):
    if 'Camera' in sensor['type']:
        bpy.ops.object.add(type='CAMERA', location=sensor['pose']['translation'],
                           rotation=sensor['pose']['rotation'],
                           layers=utility.defLayers([defs.layerTypes['sensor']]))
        newsensor = bpy.context.active_object
    elif sensor['type'] in ["RaySensor", "RotatingRaySensor", "ScanningSonar"]:
        # TODO: create a proper ray sensor scanning layer disc here
        newsensor = utility.createPrimitive(sensor['name'], 'disc', (0.5, 36),
                                            defs.layerTypes['sensor'], 'phobos_laserscanner',
                                            sensor['pose']['translation'], sensor['pose']['rotation'])
    else:  # contact, force and torque sensors (or unknown sensors)
        newsensor = utility.createPrimitive(sensor['name'], 'sphere', 0.05,
                                            defs.layerTypes['sensor'], 'phobos_sensor',
                                            sensor['pose']['translation'], sensor['pose']['rotation'])
        newsensor.phobostype = 'sensor'
        #newsensor.name = sensor['name'] if sensor['name'] != '' else 'new_' + sensor['type']
    # TODO: use defaults?
    #for prop in defs.sensorProperties[sensor['type']]:
    #    sensor[prop] = defs.sensorProperties[sensor['type']][prop]
    for prop in sensor:
        if prop not in ['name', 'pose']:
            newsensor[prop] = sensor[prop]
    if sensor['type'] not in defs.sensortypes:
        print("### Warning: sensor", sensor['name'], "is of unknown type.")
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

    def execute(self, context):
        sensor = None
        sensortype = self.custom_type if self.sensor_type == 'Custom' else self.sensor_type
        if 'Node' in sensortype or 'Joint' in sensortype or 'Motor' in sensortype:
            objects = []
            sensorobjects = []
            for obj in context.selected_objects:
                if obj.phobostype == 'sensor':
                    sensorobjects.append(obj)
                else:
                    objects.append(obj)
            if len(sensorobjects) > 0:
                for sensor in sensorobjects:
                    if 'Node' in sensor['sensor/type']:
                        sensor['sensor/nodes'] = [obj.name for obj in objects if obj.phobostype == 'collision']
                    elif 'Joint' in sensor['sensor/type'] or "Motor" in sensor['sensor/type']:
                        sensor['sensor/joints'] = [obj.name for obj in objects if obj.phobostype == 'link']
            else:
                sensor = {'type': sensortype, 'name': self.sensor_name,
                          'pose': {'translation': list(context.scene.cursor_location),
                                   'rotation': [0, 0, 0]}}
        else:
            sensor = {'type': sensortype, 'name': self.sensor_name,
                      'pose': {'translation': list(context.scene.cursor_location),
                               'rotation': [0, 0, 0]}}
        if sensor is not None:
            createSensor(sensor)
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
