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
    """ExportModelOperator"""
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
            mtutility.createPrimitive(self.sensor_type, self.sensor_type, 10, location, mtdefs.layerTypes["sensors"], "sensor")
            sensor = bpy.context.scene.objects.active
            if "Node" in self.sensor_type:
                #TODO; add sensor nodes
                print("Added nodes to new " + self.sensor_type)
            elif "Joint" in self.sensor_type:
                #TODO; add sensor joints
                print("Added nodes to new " + self.sensor_type)
            elif "Motor" in self.sensor_type:
                #TODO; add sensor joints
                print("Added nodes to new " + self.sensor_type)
            else: #visual sensor
                if self.sensor_type == "RaySensor":
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "CameraSensor":
                    print("Added nodes to new " + self.sensor_type)
                elif self.sensor_type == "ScanningSonar":
                    print("Added nodes to new " + self.sensor_type)
        else:
            print("Sensor could not be created: unknown sensor type.")
        return {'FINISHED'}


    sensorTypes = ["RaySensor",
               "CameraSensor",
               "ScanningSonar",

               "JointPosition",
               "JointVelocity",
               "JointLoad",
               "JointTorque",
               "JointAVGTorque",
               "Joint6DOF",

               "NodeContact",
               "NodePosition",
               "NodeRotation",
               "NodeContactForce",
               "NodeCOM",
               "NodeVelocity",
               "NodeAngularVelocity",

               "MotorCurrent"
              ]


