'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtsensors.py

Created on 6 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''


import bpy
import marstools.mtmaterials as mtmaterials
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility
from bpy.types import Operator
from bpy.props import StringProperty, FloatProperty

def register():
    print("Registering mtsensors...")

def unregister():
    print("Unregistering mtsensors...")



class AddSensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.mt_add_sensor"
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
        if self.sensor_type in mtdefs.sensorTypes:
            if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
                objects = []
                sensors = []
                for obj in bpy.context.selected_objects:
                    if obj.MARStype == "sensor":
                        sensors.append(obj)
                        self.sensor_type = obj["sensorType"]
                    else:
                        objects.append(obj)
                if len(sensors) <= 0:
                    mtutility.createPrimitive(self.sensor_type, "sphere", self.sensor_scale, mtdefs.layerTypes["sensor"], "sensor", location)
                    sense = bpy.context.scene.objects.active
                    sense.MARStype = "sensor"
                    sense.name = self.sensor_type
                    sense["sensorType"] = self.sensor_type
                    sensors.append(sense)
                for sensor in sensors:
                    for key in sensor.keys():
                        if key.find("index") >= 0:
                            del sensor[key]
                            print("Deleting " + key + " in " + sensor.name)
                    i = 1
                    if "Node" in sensor["sensorType"]:
                        for obj in objects:
                            if obj.MARStype == "collision":
                                sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                                i += 1
                        print("Added nodes to new " + self.sensor_type)
                    elif "Joint" in sensor["sensorType"] or "Motor" in sensor["sensorType"]:
                        for obj in objects:
                            if obj.MARStype == "link":
                                sensor["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                                i += 1
                        print("Added nodes to new " + self.sensor_type)
                    if len(objects) > 0:
                        sensor.parent = mtutility.getRoot(objects[0])
            else: #visual sensor
                if self.sensor_type == "RaySensor":
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "CameraSensor":
                    bpy.ops.object.add(type='CAMERA', location=bpy.context.scene.cursor_location)
                    sensor = bpy.context.active_object
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "ScanningSonar":
                    print("Added nodes to new " + self.sensor_type)
            for prop in mtdefs.sensorProperties[self.sensor_type]:
                sensor[prop] = mtdefs.sensorProperties[self.sensor_type][prop]
        else:
            print("Sensor could not be created: unknown sensor type.")
        return {'FINISHED'}
