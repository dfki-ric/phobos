#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.selection
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators for selecting and finding objects.

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert, Simon Reichel

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
"""

import sys
import inspect

import bpy
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty
import phobos.defs as defs
import phobos.utils.selection as sUtils
from phobos.phoboslog import log


class SelectObjectsByPhobosType(Operator):
    """Select objects in the scene by phobostype"""
    bl_idname = "phobos.select_objects_by_phobostype"
    bl_label = "Select by Phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    seltype = EnumProperty(
        items=defs.phobostypes,
        name="Phobostype",
        default="link",
        description="Phobos object type")

    def execute(self, context):
        sUtils.selectObjects(sUtils.getObjectsByPhobostypes([self.seltype]), True)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'


class SelectObjectsByName(Operator):
    """Select objects in the scene by their name"""
    bl_idname = "phobos.select_objects_by_name"
    bl_label = "Select by Name"
    bl_options = {'REGISTER', 'UNDO'}

    namefragment = StringProperty(
        name="Name Contains",
        default='',
        description="Part of a Phobos object name")

    def execute(self, context):
        sUtils.selectByName(self.namefragment)
        return {'FINISHED'}


class GotoObjectOperator(Operator):
    """Selection operator for buttons to jump to the specified object"""
    bl_idname = "phobos.goto_object"
    bl_label = "Goto Object"
    bl_options = {'REGISTER', 'UNDO'}

    objectname = StringProperty(
        name="Object Name",
        default='',
        description="The name of the object to jump to")

    def execute(self, context):
        bpy.ops.object.select_all(action='DESELECT')
        context.scene.objects[self.objectname].select = True
        context.scene.objects.active = context.scene.objects[self.objectname]
        return {'FINISHED'}


class SelectRootOperator(Operator):
    """Select root object(s) of currently selected object(s)"""
    bl_idname = "phobos.select_root"
    bl_label = "Select Roots"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        roots = set()

        # add root object of each selected object
        for obj in bpy.context.selected_objects:
            roots.add(sUtils.getRoot(obj))

        # select all found root objects
        if len(roots) > 0:
            sUtils.selectObjects(list(roots), True)
            bpy.context.scene.objects.active = list(roots)[0]
        else:
            log("Couldn't find any root object.", "ERROR")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(bpy.context.selected_objects) > 0


class SelectModelOperator(Operator):
    """Select all objects of model(s) containing the currently selected object(s)"""
    bl_idname = "phobos.select_model"
    bl_label = "Select Model"
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="Model Name",
        default="",
        description="Name of the model to be selected")

    def execute(self, context):
        selection = []
        if self.modelname:
            log("phobos: Selecting model" + self.modelname, "INFO")
            roots = sUtils.getRoots()
            for root in roots:
                if root["modelname"] == self.modelname:
                    selection = sUtils.getChildren(root)
        else:
            log("No model name provided, deriving from selection...", "INFO")
            roots = set()
            for obj in bpy.context.selected_objects:
                print("Selecting", sUtils.getRoot(obj).name)
                roots.add(sUtils.getRoot(obj))
            for root in list(roots):
                selection.extend(sUtils.getChildren(root))
        sUtils.selectObjects(list(selection), True)
        return {'FINISHED'}


def register():
    print("Registering operators.selection...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    print("Unregistering operators.selection...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)
