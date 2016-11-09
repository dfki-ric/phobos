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

import bpy
from bpy.types import Operator
from bpy.props import BoolProperty, StringProperty
import phobos.defs as defs
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
from phobos.logging import startLog, endLog, log


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

    def execute(self, context):
        startLog(self)
        objlist = context.selected_objects
        if self.complete:
            roots = list(set([sUtils.getRoot(obj) for obj in context.selected_objects]))
            if None in roots:
                roots.remove(None)
            objlist = [elem for sublist in [sUtils.getChildren(root) for root in roots] for elem in sublist]
        objnames = [o.name for o in bpy.data.objects]
        for obj in objlist:
            if "::" in obj.name:
                if nUtils.namesAreExplicit({obj.name.split("::")[-1]}, objnames):
                    nUtils.removeNamespace(obj)
                else:
                    log("Cannot remove namespace from " + obj.name + ". Name wouldn't be explicit", "ERROR")
            else:
                nUtils.addNamespace(obj)
        endLog()
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
        startLog(self)
        root = sUtils.getRoot(context.active_object)
        if root:
            root["modelname"] = self.modelname
        else:
            log("Could not set modelname due to missing root link. No name was set.", "ERROR")
        endLog()
        return {'FINISHED'}


class VersionModelOperator(Operator):
    """Set model version by assigning 'version' property to root node"""
    bl_idname = "phobos.set_version"
    bl_label = "Set Version of Model"
    bl_options = {'REGISTER', 'UNDO'}

    version = StringProperty(
        name="Version",
        default="",
        description="Version of the robot model to be assigned")

    def execute(self, context):
        startLog(self)
        root = sUtils.getRoot(context.active_object)
        if root:
            root["version"] = self.version
        else:
            log("Could not set version due to missing root link. No name was set.", "ERROR")
        endLog()
        return {'FINISHED'}


class PartialRename(Operator):
    """Replace part of the name of selected object(s)"""
    bl_idname = "phobos.partial_rename"
    bl_label = "Partial Rename"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name="Find",
        default="",
        description="Find string")

    replace = StringProperty(
        name="Replace",
        default="",
        description="Replace with")

    def execute(self, context):
        types = defs.subtypes
        for obj in context.selected_objects:
            obj.name = obj.name.replace(self.find, self.replace)
            for type in types:
                nametag = type + "/name"
                if nametag in obj:
                    obj[nametag] = obj[nametag].replace(self.find, self.replace)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0
