#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.io
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators import/export

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

from phobos.logging import startLog, endLog, log
import phobos.defs as defs
import phobos.exporter as exporter
import phobos.importer as importer
import phobos.links as links
import bpy
import yaml
import os
import phobos.utils.selection as sUtils
import phobos.robotdictionary as robotdictionary
from bpy.types import Operator
from bpy.props import EnumProperty, StringProperty

def generateLibEntries(param1, param2): #FIXME: parameter?
    with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
        return [("None",)*3] + [(entry,)*3 for entry in yaml.load(f.read())]


class ImportLibRobot(Operator):
    """Import a baked robot into the robot library"""
    bl_idname = "object.phobos_import_lib_robot"
    bl_label = "Import Baked Robot"
    bl_options = {'REGISTER', 'UNDO'}

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        startLog(self)
        libPath = os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml")
        path, file = os.path.split(self.filepath)
        if file.endswith(".bake"):
            with open(self.filepath, "r") as f:
                info = yaml.load(f.read())
            if not os.path.isfile(libPath):
                open(libPath, "a").close()
            with open(libPath, "r+") as f:
                robot_lib = yaml.load(f.read())
                robot_lib = robot_lib if robot_lib is not None else {}
                robot_lib[info["name"]] = path
                f.seek(0)
                f.write(yaml.dump(robot_lib))
                f.truncate()
        else:
            log("This is no robot bake!", "ERROR")
        endLog()
        return {"FINISHED"}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}


class CreateRobotInstance(Operator):
    """Create a new instance of the selected robot lib entry"""
    bl_idname = "object.phobos_create_robot_instance"
    bl_label = "Create Robot Instance"
    bl_options = {'REGISTER', 'UNDO'}

    bakeObj = EnumProperty(
        name="Robot Lib Entries",
        items=generateLibEntries,
        description="The robot lib entries")

    robName = StringProperty(
        name="Instance Name",
        default="instance",
        description="The instance's name"
    )

    def execute(self, context):
        if self.bakeObj == "None":
            return {"FINISHED"}
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robot_lib = yaml.load(f.read())
        root = links.createLink(1.0, name=self.robName + "::" + self.bakeObj)
        root["modelname"] = self.bakeObj
        root["entity/name"] = self.robName
        root["isInstance"] = True
        bpy.ops.import_mesh.stl(filepath=os.path.join(robot_lib[self.bakeObj], "bake.stl"))
        bpy.ops.view3d.snap_selected_to_cursor(use_offset=False)
        obj = context.active_object
        obj.name = self.robName + "::visual"
        obj.phobostype = "visual"
        sUtils.selectObjects([root, obj], clear=True, active=0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        return {"FINISHED"}

    @classmethod
    def poll(self, context):
        return os.path.isfile(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"))


class ExportSceneOperator(Operator):
    """Export the selected model(s) in a scene"""
    bl_idname = "object.phobos_export_scene"
    bl_label = "Export Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        exporter.exportSMURFsScene()
        endLog()
        return {'FINISHED'}


class ExportModelOperator(Operator):
    """Export the selected model(s)"""
    bl_idname = "object.phobos_export_robot"
    bl_label = "Export Model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        root = sUtils.getRoot(context.selected_objects[0])
        if root.phobostype != 'link':
            log("Selection includes objects not parented to any model root, please adapt selection.", "ERROR", "ExportModelOperator")
        else:
            model, objectlist = robotdictionary.buildModelDictionary(root)
            exporter.export(model, objectlist)
            endLog()
        return {'FINISHED'}


class ExportBakeOperator(Operator):
    """Bake the selected model"""
    bl_idname = "object.phobos_export_bake"
    bl_label = "Export Bake"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        objs = context.selected_objects
        root = sUtils.getRoot(context.selected_objects[0])
        model = robotdictionary.buildModelDictionary(root)
        sUtils.selectObjects(objs)
        if bpy.data.worlds[0].relativePath:
            outpath = exporter.securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
        else:
            outpath = exporter.securepath(os.path.expanduser(bpy.data.worlds[0].path))
        exporter.bakeModel(objs, outpath, model["modelname"])
        with open(os.path.join(outpath, "info.bake"), "w") as f:
            f.write(yaml.dump({"name": model["modelname"]}))
        endLog()
        return {'FINISHED'}


class RobotModelImporter(bpy.types.Operator):
    """Import robot model file from various formats"""
    bl_idname = "obj.import_robot_model"
    bl_label = ""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        modeltype = self.filepath.split('.')[-1]
        if modeltype == 'scene':
            imp = importer.MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            imp = importer.URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            imp = importer.SMURFModelParser(self.filepath)
        elif modeltype == 'scn':
            imp = importer.MARSModelParser(self.filepath, zipped=True)
        else:
            print("Unknown model format, aborting import...")
        importer.cleanUpScene()
        imp.parseModel()
        imp.createBlenderModel()
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
