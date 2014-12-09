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

File controllers.py

Created on 30 Jan 2014

@author: Kai von Szadkowski
"""

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

    controller_rate = FloatProperty(
        name = "rate",
        default = 10,
        description = "rate of the controller [Hz]")

    def execute(self, context):
        location = bpy.context.scene.cursor_location
        objects = []
        controllers = []
        for obj in bpy.context.selected_objects:
            if obj.phobostype == "controller":
                controllers.append(obj)
            else:
                objects.append(obj)
        if len(controllers) <= 0:
            utility.createPrimitive("controller", "sphere", self.controller_scale, defs.layerTypes["sensor"], "controller", location)
            bpy.context.scene.objects.active.phobostype = "controller"
            bpy.context.scene.objects.active.name = "controller"
            controllers.append(bpy.context.scene.objects.active)
        #empty index list so enable robotupdate of controller
        for ctrl in controllers:
            sensors = [obj.name for obj in objects if obj.phobostype == 'sensor']
            joints = [obj.name for obj in objects if obj.phobostype == 'link' and not 'joint/passive' in obj]
            ctrl['controller/sensors'] = sorted(sensors, key=str.lower)
            ctrl['controller/motors'] = sorted(joints, key=str.lower)
            ctrl['controller/rate'] = self.controller_rate
        print("Added joints/motors to (new) controller(s).")
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
            if obj.phobostype == "controller":
                controllers.append(obj)
            else:
                objects.append(obj)
        if len(controllers) <= 0:
            utility.createPrimitive("controller", "sphere", self.controller_scale, defs.layerTypes["sensor"], "controller", location)
            bpy.context.scene.objects.active.phobostype = "controller"
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
                if obj.phobostype == "link":
                    ctrl["index"+(str(i) if i >= 10 else "0"+str(i))] = obj.name
                    i += 1
        print("Added joints to (new) controller(s).")
        #for prop in defs.controllerProperties[self.controller_type]:
        #    for ctrl in controllers:
        #        ctrl[prop] = defs.controllerProperties[prop]
        return {'FINISHED'}
