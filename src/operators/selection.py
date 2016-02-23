#!/usr/bin/python

"""
.. module:: phobos.operators.selection
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators for selecting and finding objects.

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert

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

import bpy
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty
import phobos.defs as defs
from phobos.utils.selection import *
from phobos.logging import startLog, endLog, log


class SelectObjectsByPhobosType(Operator):
    """Select objects in the scene by phobostype"""
    bl_idname = "object.phobos_select_objects_by_phobostype"
    bl_label = "Select by Phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    seltype = EnumProperty(
        items=defs.phobostypes,
        name="Phobostype",
        default="link",
        description="Phobos object type")

    def execute(self, context):
        selectObjects(getObjectsByPhobostypes([self.seltype]), True)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'


class SelectObjectsByName(Operator):
    """Select objects in the scene by their name"""
    bl_idname = "object.phobos_select_objects_by_name"
    bl_label = "Select by Name"
    bl_options = {'REGISTER', 'UNDO'}

    namefragment = StringProperty(
        name="Name Contains",
        default='',
        description="Part of a Phobos object name")

    def execute(self, context):
        selectByName(self.namefragment)
        return {'FINISHED'}


class SelectRootOperator(Operator):
    """Select root object(s) of currently selected object(s)"""
    bl_idname = "object.phobos_select_root"
    bl_label = "Select Roots"

    def execute(self, context):
        startLog(self)
        roots = set()
        for obj in bpy.context.selected_objects:
            roots.add(getRoot(obj))
        if len(roots) > 0:
            selectObjects(list(roots), True)
            bpy.context.scene.objects.active = list(roots)[0]
        else:
            # bpy.ops.error.message('INVOKE_DEFAULT', type="ERROR", message="Couldn't find any root object.")
            log("Couldn't find any root object.", "ERROR")
        endLog()
        return {'FINISHED'}


class SelectModelOperator(Operator):
    """Select all objects of model(s) containing the currently selected object(s)"""
    bl_idname = "object.phobos_select_model"
    bl_label = "Select Model"
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="Model Name",
        default="",
        description="Name of the model to be selected")

    def execute(self, context):
        selection = []
        if self.modelname:
            print("phobos: Selecting model", self.modelname)
            roots = getRoots()
            for root in roots:
                if root["modelname"] == self.modelname:
                    selection = getChildren(root)
        else:
            print("phobos: No model name provided, deriving from selection...")
            roots = set()
            for obj in bpy.context.selected_objects:
                print("Selecting", getRoot(obj).name)
                roots.add(getRoot(obj))
            for root in list(roots):
                selection.extend(getChildren(root))
        selectObjects(list(selection), True)
        return {'FINISHED'}
