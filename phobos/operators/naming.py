#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.naming
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators for naming objects

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

import sys
import inspect

import bpy
from bpy.types import Operator
from bpy.props import BoolProperty, StringProperty
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.io as iUtils
from phobos.phoboslog import log


class ToggleNamespaces(Operator):
    """Toggle the use of namespaces for the selected objects"""
    bl_idname = "phobos.toggle_namespaces"
    bl_label = "Toggle Namespaces"
    bl_options = {'REGISTER', 'UNDO'}

    complete = BoolProperty(
        name="Convert Complete Robot",
        default=False,
        description="Convert the complete robot"
    )

    namespace = StringProperty()

    def execute(self, context):
        if self.complete:
            roots = set([sUtils.getRoot(obj) for obj in context.selected_objects]) - {None}
            objects = set()
            for root in roots:
                objects = objects | set(sUtils.getChildren(root))
            objlist = list(objects)
        else:
            objlist = [bpy.context.active_object]
        for obj in objlist:
            try:
                entityname = sUtils.getRoot(obj)['entity/name']
            except (KeyError, TypeError):
                entityname = ''
                log(nUtils.getObjectName(obj) + " is not part of a well-defined entity.", "WARNING")
            namespace = self.namespace if self.namespace else entityname
            nUtils.toggleNamespace(obj, namespace)
        return {'FINISHED'}


class NameModelOperator(Operator):
    """Name model by assigning 'modelname' property to root node"""
    bl_idname = "phobos.name_model"
    bl_label = "Name Model"
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="Model Name",
        default="",
        description="Name of the robot model to be assigned")

    def execute(self, context):
        root = sUtils.getRoot(context.active_object)
        if root:
            root["modelname"] = self.modelname
        else:
            log("Could not set modelname due to missing root link. No name was set.", "ERROR")
        return {'FINISHED'}


class SetModelVersionOperator(Operator):
    """Set model version by assigning 'version' property to root node"""
    bl_idname = "phobos.set_version"
    bl_label = "Set Model Version"
    bl_options = {'REGISTER', 'UNDO'}

    version = StringProperty(
        name="Version",
        default="",
        description="Version of the model to be assigned")

    usegitbranch = BoolProperty(
        name="Use Git branch name",
        default=False,
        description="Insert Git branch name in place of *?")

    def execute(self, context):
        root = sUtils.getRoot(context.active_object)
        if root:
            if self.usegitbranch:
                gitbranch = iUtils.getgitbranch()
                if gitbranch:
                    root["version"] = self.version.replace('*', gitbranch)
            else:
                root["version"] = self.version
        else:
            log("Could not set version due to missing root link. No version was set.", "ERROR")
        return {'FINISHED'}


class BatchRename(Operator):
    """Replace part of the name of selected object(s)"""
    bl_idname = "phobos.batch_rename"
    bl_label = "Batch Rename"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name="Find:",
        default="",
        description="A string to be replaced.")

    replace = StringProperty(
        name="Replace:",
        default="",
        description="A string to replace the 'Find' string.")

    add = StringProperty(
        name="Add/Embed:",
        default="*",
        description="Add any string by representing the old name with '*'.")

    include_properties = BoolProperty(
        name="Include Properties",
        default=False,
        description="Replace names stored in '*/name' properties?")

    def execute(self, context):
        for obj in context.selected_objects:
            obj.name = self.add.replace('*', obj.name.replace(self.find, self.replace))
            if self.include_properties:
                for key in obj.keys():
                    if key.endswith('/name'):
                        obj[key] = self.add.replace('*', obj[key].replace(self.find, self.replace))
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


def register():
    print("Registering operators.naming...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    print("Unregistering operators.naming...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)
