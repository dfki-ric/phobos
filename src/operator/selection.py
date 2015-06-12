#!/usr/bin/python

"""
.. module:: phobos.operator.selection
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


class SelectObjectsByPhobosType(Operator):
    """SelectObjectsByPhobosType

    """
    bl_idname = "object.phobos_select_objects_by_phobostype"
    bl_label = "Select objects in the scene by phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    seltype = EnumProperty(
        items=defs.phobostypes,
        name="phobostype",
        default="link",
        description="Phobos object type")

    def execute(self, context):
        objlist = []
        for obj in bpy.data.objects:
            if obj.phobostype == self.seltype:
                objlist.append(obj)
        utility.selectObjects(objlist, True)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'


class SelectObjectsByName(Operator):
    """SelectObjectsByName

    """
    bl_idname = "object.phobos_select_objects_by_name"
    bl_label = "Select objects in the scene by their name"
    bl_options = {'REGISTER', 'UNDO'}

    namefragment = StringProperty(
        name="name contains",
        default='',
        description="part of a Phobos object name")

    def execute(self, context):
        utility.selectByName(self.namefragment)
        return {'FINISHED'}


class SelectRootOperator(Operator):
    """SelectRootOperator

    """
    bl_idname = "object.phobos_select_root"
    bl_label = "Select root object(s) of currently selected object(s)"

    def execute(self, context):
        startLog(self)
        roots = set()
        for obj in bpy.context.selected_objects:
            roots.add(utility.getRoot(obj))
        if len(roots) > 0:
            utility.selectObjects(list(roots), True)
            bpy.context.scene.objects.active = list(roots)[0]
        else:
            # bpy.ops.error.message('INVOKE_DEFAULT', type="ERROR", message="Couldn't find any root object.")
            log("Couldn't find any root object.", "ERROR")
        endLog()
        return {'FINISHED'}


class SelectModelOperator(Operator):
    """SelectModelOperator

    """
    bl_idname = "object.phobos_select_model"
    bl_label = "Select all objects of model(s) containing the currently selected object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="modelname",
        default="",
        description="name of the model to be selected")

    def execute(self, context):
        selection = []
        if self.modelname:
            print("phobos: Selecting model", self.modelname)
            roots = utility.getRoots()
            for root in roots:
                if root["modelname"] == self.modelname:
                    selection = utility.getChildren(root)
        else:
            print("phobos: No model name provided, deriving from selection...")
            roots = set()
            for obj in bpy.context.selected_objects:
                print("Selecting", utility.getRoot(obj).name)
                roots.add(utility.getRoot(obj))
            for root in list(roots):
                selection.extend(utility.getChildren(root))
        utility.selectObjects(list(selection), True)
        return {'FINISHED'}