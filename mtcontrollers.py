'''
Created on 30 Jan 2014

@author: kavonszadkowski
'''

import bpy
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatProperty
import marstools.mtcreateprops as mtcreateprops
import marstools.mtmaterials as mtmaterials
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility

def register():
    print("Registering mtcontrollers...")

def unregister():
    print("Unregistering mtcontrollers...")

class AddControllerOperator(Operator):
    """AddControllerOperator"""
    bl_idname = "object.mt_add_controller"
    bl_label = "Add a node-dependent controller"
    bl_options = {'REGISTER', 'UNDO'}

    controller_scale = FloatProperty(
        name = "controller_scale",
        default = 0.1,
        description = "scale of the controller visualization")

    def execute(self, context):
        location = bpy.context.scene.cursor_location
        objects = []
        controllers = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "controller":
                controllers.append(obj)
            else:
                objects.append(obj)
        if len(controllers) <= 0:
            mtutility.createPrimitive("controller", "sphere", (self.controller_scale,), mtdefs.layerTypes["sensors"], "controller", location)
            bpy.context.scene.objects.active.MARStype = "controller"
            bpy.context.scene.objects.active.name = "controller"
            controllers.append(bpy.context.scene.objects.active)
        #empty index list so enable update of controller
        for ctrl in controllers:
            for key in ctrl.keys():
                if key.find("index") >= 0:
                    del ctrl[key]
                    print("Deleting " + key + " in " + ctrl.name)
            i = 1
            for obj in objects:
                if obj.MARStype == "joint":
                    ctrl["index"+str(i)] = obj.name
                    i += 1
        print("Added joints to (new) controller(s).")
        #for prop in mtdefs.controllerProperties[self.controller_type]:
        #    for ctrl in controllers:
        #        ctrl[prop] = mtdefs.controllerProperties[prop]
        return {'FINISHED'}