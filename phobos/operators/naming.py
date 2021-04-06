#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains all Blender operators for naming of models/objects/properties.
"""

import sys
import inspect

import bpy
from bpy.types import Operator
from bpy.props import BoolProperty, StringProperty, EnumProperty
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.io as ioUtils
import phobos.utils.validation as validation
from phobos.phoboslog import log


class ToggleNamespaces(Operator):
    """Toggle the use of namespaces for the selected objects"""

    bl_idname = "phobos.toggle_namespaces"
    bl_label = "Toggle Namespaces"
    bl_options = {'REGISTER', 'UNDO'}

    complete : BoolProperty(
        name="Convert Complete Robot", default=False, description="Convert the complete robot"
    )

    namespace : StringProperty()

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
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

    modelname : StringProperty(
        name="Model Name", default="", description="Name of the robot model to be assigned"
    )

    @classmethod
    def poll(cls, context):
        """Hide operator if there is no link present.

        Args:
          context: 

        Returns:

        """
        root = sUtils.getRoot(context.active_object, verbose=False)
        return root and root.phobostype == 'link'

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        root = sUtils.getRoot(context.active_object)
        if root:
            root["model/name"] = self.modelname
            # write model information to new root
            root.pose.bones[0].custom_shape = ioUtils.getResource(('link', 'root'))
        else:
            log("Could not set modelname due to missing root link. No name was set.", "ERROR")
        return {'FINISHED'}


class SetModelVersionOperator(Operator):
    """Set model version by assigning 'version' property to root node"""

    bl_idname = "phobos.set_version"
    bl_label = "Set Model Version"
    bl_options = {'REGISTER', 'UNDO'}

    version : StringProperty(
        name="Version", default="", description="Version of the model to be assigned"
    )

    usegitbranch : BoolProperty(
        name="Use Git branch name",
        default=False,
        description="Insert Git branch name in place of *?",
    )

    @classmethod
    def poll(cls, context):
        """Hide operator if there is no link present.

        Args:
          context: 

        Returns:

        """
        root = sUtils.getRoot(context.active_object, verbose=False)
        return root and root.phobostype == 'link'

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        root = sUtils.getRoot(context.active_object)
        if self.usegitbranch:
            gitbranch = ioUtils.getgitbranch()
            if gitbranch:
                root["model/version"] = self.version.replace('*', gitbranch)
        else:
            root["model/version"] = self.version
        return {'FINISHED'}


class BatchRename(Operator):
    """Replace part of the name of selected object(s)"""

    bl_idname = "phobos.batch_rename"
    bl_label = "Batch Rename"
    bl_options = {'REGISTER', 'UNDO'}

    find : StringProperty(name="Find:", default="", description="A string to be replaced.")

    replace : StringProperty(
        name="Replace:", default="", description="A string to replace the 'Find' string."
    )

    add : StringProperty(
        name="Add/Embed:",
        default="*",
        description="Add any string by representing the old name with '*'.",
    )

    include_properties : BoolProperty(
        name="Include Properties",
        default=False,
        description="Replace names stored in '*/name' properties?",
    )

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        for obj in context.selected_objects:
            obj.name = self.add.replace('*', obj.name.replace(self.find, self.replace))
            if self.include_properties:
                for key in obj.keys():
                    if key.endswith('/name'):
                        obj[key] = self.add.replace('*', obj[key].replace(self.find, self.replace))
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return len(context.selected_objects) > 0


class FixObjectNames(Operator):
    """Cleans up the redundant names of the active object"""

    bl_idname = "phobos.fix_object_names"
    bl_label = "Rename Object"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        obj = context.active_object
        errors = validation.validateObjectNames(obj)

        for error in errors:
            if error.message[:9] == 'Redundant':
                log("Deleting redundant name '" + error.information + "'.", 'INFO')
                del obj[error.information]

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return context.active_object


class ChangeObjectName(Operator):
    """Changes the name of the object"""

    bl_idname = "phobos.change_object_name"
    bl_label = "Change Object Name"
    bl_options = {'REGISTER', 'UNDO'}

    newname : StringProperty(name="New name", description="New name of the object", default="")

    jointname : StringProperty(name="Joint name", description="Name of the joint", default="")

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        obj = context.active_object

        # rename only if necessary
        if self.newname != '' and self.newname != nUtils.getObjectName(obj):
            log(
                "Renaming "
                + obj.phobostype
                + " '"
                + nUtils.getObjectName(obj)
                + "' to '"
                + self.newname
                + "'.",
                'INFO',
            )
            nUtils.safelyName(obj, self.newname)
        elif self.newname == '':
            log("Removing custom name from " + obj.phobostype + " '" + obj.name + "'.", 'INFO')
            if obj.phobostype + '/name' in obj:
                del obj[obj.phobostype + '/name']

        # only links have joint names
        if obj.phobostype == 'link':
            if self.jointname != '':
                # only change/add joint/name if it was changed
                if 'joint/name' not in obj or (
                    'joint/name' in obj and self.jointname != obj['joint/name']
                ):
                    log(
                        "Renaming joint of "
                        + obj.phobostype
                        + " '"
                        + nUtils.getObjectName(obj)
                        + "' to '"
                        + self.jointname
                        + "'.",
                        'INFO',
                    )
                    obj['joint/name'] = self.jointname
            # remove joint/name when empty
            elif self.jointname == '':
                if 'joint/name' in obj:
                    log(
                        "Removing joint name from " + obj.phobostype + " '" + obj.name + "'.",
                        'INFO',
                    )
                    del obj['joint/name']

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context: 

        Returns:

        """
        return context.active_object and context.active_object.phobostype != 'undefined'

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        wm = context.window_manager
        obj = context.active_object

        self.newname = nUtils.getObjectName(obj)
        if 'joint/name' in obj:
            self.jointname = obj['joint/name']
        return wm.invoke_props_dialog(self)

    def draw(self, context):
        """

        Args:
          context: 

        Returns:

        """
        obj = context.active_object
        layout = self.layout

        if obj.phobostype == 'link':
            layout.prop(self, 'newname', text="Link name")
            layout.prop(self, 'jointname')
        else:
            layout.prop(self, 'newname')
            self.jointname = ''
            layout.label(text="Phobostype: " + obj.phobostype)

classes = (
    ToggleNamespaces,
    NameModelOperator,
    SetModelVersionOperator,
    BatchRename,
    FixObjectNames,
    ChangeObjectName,
)
def register():
    """TODO Missing documentation"""
    print("Registering operators.naming...")
    for classdef in classes:
        bpy.utils.register_class(classdef)


def unregister():
    """TODO Missing documentation"""
    print("Unregistering operators.naming...")
    for classdef in classes:
        bpy.utils.unregister_class(classdef)
