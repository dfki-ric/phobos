'''
Created on 9 Jan 2014

@author: kavonszadkowski
'''


import bpy
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty
import marstools.mtcreateprops as mtcreateprops
import marstools.mtexport as mtexport
import marstools.mtmaterials as mtmaterials
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility

def register():
    print("Registering mtsensors...")

def unregister():
    print("Unregistering mtsensors...")



class AddSensorOperator(Operator):
    """AddSensorOperator"""
    bl_idname = "object.mt_add_sensor"
    bl_label = "Add a node-dependent sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = StringProperty(
        name = "sensor_type",
        default = "",
        description = "type of the sensor to be created")

    test = "123"

    def execute(self, context):
        location = bpy.context.scene.cursor_location
        if self.sensor_type in mtdefs.sensorTypes:
            if "Node" in self.sensor_type or "Joint" in self.sensor_type or "Motor" in self.sensor_type:
                i = 1
                objects = []
                for obj in bpy.context.selected_objects:
                    objects.append(obj)
                mtutility.createPrimitive(self.sensor_type, "sphere", 0.1, location, mtdefs.layerTypes["sensors"], "sensor")
                sensor = bpy.context.scene.objects.active
                sensor.MARStype = "sensor"
                if "Node" in self.sensor_type:
                    for obj in objects:
                        if obj.MARStype == "body":
                            sensor["index"+str(i)] = obj.name
                            i += 1
                    print("Added nodes to new " + self.sensor_type)
                elif "Joint" in self.sensor_type:
                    for obj in objects:
                        if obj.MARStype == "joint":
                            sensor["index"+str(i)] = obj.name
                            i += 1
                    print("Added nodes to new " + self.sensor_type)
                elif "Motor" in self.sensor_type:
                    for obj in objects:
                        if obj.MARStype == "joint":
                            sensor["index"+str(i)] = obj.name
                            i += 1
                    print("Added nodes to new " + self.sensor_type)
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
                sensor.name = self.sensor_type
                sensor[prop] = mtdefs.sensorProperties[self.sensor_type][prop]
        else:
            print("Sensor could not be created: unknown sensor type.")
        return {'FINISHED'}

