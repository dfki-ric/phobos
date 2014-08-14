'''
Phobos - a Blender Add-On to work with MARS robot models

File controllers.py

Created on 30 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatProperty
from . import defs
from . import utility


def register():
    print("Registering controllers...")


def unregister():
    print("Unregistering controllers...")


class AddControllerOperator(Operator):
    """AddControllerOperator"""
    bl_idname = "object.phobos_add_controller"
    bl_label = "Add a node-dependent controller"
    bl_options = {'REGISTER', 'UNDO'}

    controller_scale = FloatProperty(
        name = "controller scale",
        default = 0.05,
        description = "scale of the controller visualization")

    controller_name = StringProperty(
        name = "controller name",
        default = 'controller',
        description = "name of the controller")

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
            utility.createPrimitive("controller", "sphere", self.controller_scale, defs.layerTypes["sensor"], "controller", location)
            bpy.context.scene.objects.active.MARStype = "controller"
            bpy.context.scene.objects.active.name = "controller"
            controllers.append(bpy.context.scene.objects.active)
        #empty index list so enable robotupdate of controller
        for ctrl in controllers:
            sensors = [obj.name for obj in objects if obj.MARStype == 'sensor']
            motors = [obj.name for obj in objects if obj.MARStype == 'motor']
            ctrl['sensors'] = sorted(sensors, key=str.lower)
            ctrl['motors'] = sorted(motors, key=str.lower)
        print("Added joints to (new) controller(s).")
        #for prop in defs.controllerProperties[self.controller_type]:
        #    for ctrl in controllers:
        #        ctrl[prop] = defs.controllerProperties[prop]
        return {'FINISHED'}


class AddLegacyControllerOperator(Operator):
    """AddControllerOperator"""
    bl_idname = "object.phobos_add_legacy_controller"
    bl_label = "Add a node-dependent controller"
    bl_options = {'REGISTER', 'UNDO'}

    controller_scale = FloatProperty(
        name = "controller_scale",
        default = 0.05,
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
            utility.createPrimitive("controller", "sphere", self.controller_scale, defs.layerTypes["sensor"], "controller", location)
            bpy.context.scene.objects.active.MARStype = "controller"
            bpy.context.scene.objects.active.name = "controller"
            controllers.append(bpy.context.scene.objects.active)
        #empty index list so enable robotupdate of controller
        for ctrl in controllers:
            for key in ctrl.keys():
                if key.find("index") >= 0:
                    del ctrl[key]
                    print("Deleting " + key + " in " + ctrl.name)
            i = 1
            for obj in objects:
                if obj.MARStype == "link":
                    ctrl["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                    i += 1
        print("Added joints to (new) controller(s).")
        #for prop in defs.controllerProperties[self.controller_type]:
        #    for ctrl in controllers:
        #        ctrl[prop] = defs.controllerProperties[prop]
        return {'FINISHED'}
