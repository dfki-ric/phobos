#!/usr/bin/python

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
import phobos.defs as defs
from phobos.logging import startLog, endLog, log
from bpy.props import BoolProperty, StringProperty
import phobos.utils.selection as selectionUtils
import phobos.utils.naming as namingUtils

class ToggleNamespaces(Operator):
    """ToggleNamespacesOperater

    """
    bl_idname = "object.phobos_toggle_namespaces"
    bl_label = "Toggles the use of namespaces for the selected objects"
    bl_options = {'REGISTER', 'UNDO'}

    complete = BoolProperty(
        name="Converting complete robot",
        default=False
    )

    def execute(self, context):
        startLog(self)
        objlist = context.selected_objects
        if self.complete:
            roots = list(set([selectionUtils.getRoot(obj) for obj in context.selected_objects]))
            if None in roots:
                roots.remove(None)
            objlist = [elem for sublist in [selectionUtils.getChildren(root) for root in roots] for elem in sublist]
        objnames = [o.name for o in bpy.data.objects]
        for obj in objlist:
            if "::" in obj.name:
                if namingUtils.namesAreExplicit({obj.name.split("::")[-1]}, objnames):
                    namingUtils.removeNamespace(obj)
                else:
                    log("Cannot remove namespace from " + obj.name + ". Name wouldn't be explicit", "ERROR")
            else:
                namingUtils.addNamespace(obj)
        endLog()
        return {'FINISHED'}


class NameModelOperator(Operator):
    """NameModelOperator

    """
    bl_idname = "object.phobos_name_model"
    bl_label = "Name model by assigning 'modelname' property to root node "
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="modelname",
        default="",
        description="name of the robot model to be assigned")

    def execute(self, context):
        startLog(self)
        root = selectionUtils.getRoot(bpy.context.active_object)
        if root == None:
            log("Could not set modelname due to missing root link. No name was set.", "ERROR")
            return {'FINISHED'}
        root["modelname"] = self.modelname
        endLog()
        return {'FINISHED'}


class PartialRename(Operator):
    """Partial Rename Operator

    """
    bl_idname = "object.phobos_partial_rename"
    bl_label = "Replace part of the name of selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name="find",
        default="",
        description="find string")

    replace = StringProperty(
        name="replace",
        default="",
        description="replace with")

    def execute(self, context):
        types = defs.subtypes
        for obj in bpy.context.selected_objects:
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
