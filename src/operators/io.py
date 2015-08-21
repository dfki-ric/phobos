#!/usr/bin/python

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
import bpy
import yaml
import zipfile
import os
import shutil
import tempfile
import phobos.utils.selection as selectionUtils
import phobos.utils.blender as blenderUtils
import phobos.robotdictionary as robotdictionary
from bpy.types import Operator
from bpy.props import EnumProperty

def generateLibEntries(param1, param2): #FIXME: parameter?
    with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
        return [("None",)*3] + [(entry,)*3 for entry in yaml.load(f.read())]

class ImportLibRobot(Operator):
    """ImportLibRobotOperator

    """
    bl_idname = "object.phobos_import_lib_robot"
    bl_label = "Imports a baked robot into the robot library."
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
    """CreateRobotInstance

    """
    bl_idname = "object.phobos_create_robot_instance"
    bl_label = "Creates a new instance of the selected robot lib entry"
    bl_options = {'REGISTER', 'UNDO'}

    bakeObj = EnumProperty(
        name="Robot lib entries",
        items=generateLibEntries,
        description="The Robot lib entries.")

    def execute(self, context):
        if self.bakeObj == "None":
            return {"FINISHED"}
        with open(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"), "r") as f:
            robot_lib = yaml.load(f.read())
        bpy.ops.import_mesh.stl(filepath=os.path.join(robot_lib[self.bakeObj], "bake.stl"))
        bpy.ops.view3d.snap_selected_to_cursor(use_offset=False)
        obj = context.active_object
        obj.name = self.bakeObj + "::instance"
        obj.phobostype = "link"
        obj["reference"] = self.bakeObj
        return {"FINISHED"}

    @classmethod
    def poll(self, context):
        return os.path.isfile(os.path.join(os.path.dirname(defs.__file__), "RobotLib.yml"))


class ExportSceneOperator(Operator):
    """This Blender operator exports the selected robot models in the current
     Blender scene as a SMURF scene (*.smurfs).
    """
    bl_idname = "object.phobos_export_scene"
    bl_label = "Export the selected model(s) in a scene."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        exporter.exportSMURFsScene()
        return {'FINISHED'}


class ExportModelOperator(Operator):
    """This blender operator exports the robot model to chosen formats.
    You can choose one or more of the following file formats:
    - SMURF
    - SRDF
    - YAML
    - MARS

    """
    bl_idname = "object.phobos_export_robot"
    bl_label = "Export the selected model(s)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        exporter.export()
        endLog()
        return {'FINISHED'}


class ExportBakeOperator(Operator):
    bl_idname = "object.phobos_export_bake"
    bl_label = "Bakes the selected model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        objs = context.selected_objects
        robot = robotdictionary.buildRobotDictionary()
        selectionUtils.selectObjects(objs)
        outpath = ""
        if bpy.data.worlds[0].relativePath:
            outpath = exporter.securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
        else:
            outpath = exporter.securepath(os.path.expanduser(bpy.data.worlds[0].path))
        #expPath = os.path.join(outpath, robot["modelname"] + "_bake")
        #exporter.export(path=expPath, robotmodel=robot)
        exporter.bakeModel(objs, outpath, robot["modelname"])
        with open(os.path.join(outpath, "info.bake"), "w") as f:
            f.write(yaml.dump({"name": robot["modelname"]}))
        endLog()
        return {'FINISHED'}

class RobotModelImporter(bpy.types.Operator):
    """Importer for MARS-compatible model or scene files

    """
    bl_idname = "obj.import_robot_model"
    bl_label = "Import robot model file from various formats"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    #filter_glob = bpy.props.StringProperty(default="*.*",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        #directory, filename = os.path.split(self.filepath)
        modeltype = self.filepath.split('.')[-1]

        if modeltype == 'scene':
            imp = importer.MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            imp = importer.URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            imp = importer.SMURFModelParser(self.filepath)
        elif modeltype == 'scn':
            imp= importer.MARSModelParser(self.filepath, zipped=True)
        else:
            print("Unknown model format, aborting import...")

        importer.cleanUpScene()
        imp.parseModel()
        imp.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}
