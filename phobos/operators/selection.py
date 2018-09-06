#!/usr/bin/python
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------------

"""
Contains all Blender operators used to select objects by different criteria.
"""

import sys
import inspect

import bpy
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty
import phobos.defs as defs
import phobos.utils.selection as sUtils
import phobos.utils.blender as bUtils
import phobos.utils.naming as nUtils
from phobos.phoboslog import log


class SelectObjectsByPhobosType(Operator):
    """Select objects in the scene by phobostype"""

    bl_idname = "phobos.select_objects_by_phobostype"
    bl_label = "Select by Phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    seltype = EnumProperty(
        items=defs.phobostypes, name="Phobostype", default="link", description="Phobos object type"
    )

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        sUtils.selectObjects(sUtils.getObjectsByPhobostypes([self.seltype]), True)
        return {'FINISHED'}

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self, width=300)

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return context.mode == 'OBJECT'


class SelectObjectsByName(Operator):
    """Select objects in the scene by their name"""

    bl_idname = "phobos.select_objects_by_name"
    bl_label = "Select by Name"
    bl_options = {'REGISTER', 'UNDO'}

    namefragment = StringProperty(
        name="Name Contains", default='', description="Part of a Phobos object name"
    )

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        sUtils.selectByName(self.namefragment)
        return {'FINISHED'}


class GotoObjectOperator(Operator):
    """Selection operator for buttons to jump to the specified object"""

    bl_idname = "phobos.goto_object"
    bl_label = "Goto Object"
    bl_options = {'UNDO', 'INTERNAL'}

    objectname = StringProperty(
        name="Object Name", default='', description="The name of the object to jump to"
    )

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return context.scene.objects and context.mode == 'OBJECT'

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        log("Jumping to object " + self.objectname + ".", 'DEBUG')

        # switch the scene if the object is anywhere else
        scene = None
        if self.objectname not in context.scene:
            for sce in bpy.data.scenes:
                if self.objectname in sce.objects:
                    scene = sce
                    break
            else:
                log("Could not find scene of object {}. Aborted.".format(self.objectname), 'ERROR')
                return {'CANCELLED'}
            bUtils.switchToScene(scene.name)

        # toggle layer to make object visible
        bUtils.setObjectLayersActive(scene.objects[self.objectname], extendlayers=True)

        sUtils.selectObjects([scene.objects[self.objectname]], clear=True, active=0)
        return {'FINISHED'}


class SelectRootOperator(Operator):
    """Select root object(s) of currently selected object(s)"""

    bl_idname = "phobos.select_root"
    bl_label = "Select Roots"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        roots = set()

        # add root object of each selected object
        for obj in context.selected_objects:
            roots.add(sUtils.getRoot(obj))

        # select all found root objects
        if roots:
            # toggle layer to make objects visible
            for root in roots:
                bUtils.setObjectLayersActive(root, extendlayers=True)

            # select objects
            sUtils.selectObjects(list(roots), True)
            context.scene.objects.active = list(roots)[0]
        else:
            log("Couldn't find any root object.", 'ERROR')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return bpy.context.selected_objects and context.mode == 'OBJECT'


class SelectModelOperator(Operator):
    """Select all objects of model(s) containing the currently selected object(s)"""

    bl_idname = "phobos.select_model"
    bl_label = "Select Model"
    bl_options = {'REGISTER', 'UNDO'}

    modelname = StringProperty(
        name="Model Name", default="", description="Name of the model to be selected"
    )

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        selection = []
        if self.modelname:
            log("phobos: Selecting model" + self.modelname, "INFO")
            roots = sUtils.getRoots()
            for root in roots:
                if nUtils.getModelName(root) == self.modelname:
                    selection = sUtils.getChildren(root)
        else:
            log("No model name provided, deriving from selection...", "INFO")
            roots = set()
            for obj in bpy.context.selected_objects:
                roots.add(sUtils.getRoot(obj))
            for root in list(roots):
                selection.extend(sUtils.getChildren(root))
        sUtils.selectObjects(list(selection), True)
        return {'FINISHED'}


def register():
    """TODO Missing documentation"""
    print("Registering operators.selection...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    """TODO Missing documentation"""
    print("Unregistering operators.selection...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)
