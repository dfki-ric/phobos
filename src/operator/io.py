#!/usr/bin/python

"""
.. module:: phobos.operator.io
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


class ImportLibRobot(Operator):
    """ImportLibRobotOperator

    """
    bl_idname = "object.phobos_import_lib_robot"
    bl_label = "Imports a baked robot into the robot library."
    bl_options = {'REGISTER', 'UNDO'}

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")
    libpath = os.path.join(os.path.dirname(__file__), "lib")

    def execute(self, context):
        startLog(self)
        file = self.filepath.split("/")[-1]
        if self.filepath.endswith(".bake"):
            zipF = zipfile.ZipFile(self.filepath, mode="r")
            zipF.extractall(path=os.path.join(self.libpath, file.split(".")[0]))
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
        items=defs.generateLibEntries,
        description="The Robot lib entries.")

    libFolder = os.path.join(os.path.dirname(__file__), "lib")

    def execute(self, context):
        bpy.ops.import_mesh.stl(filepath=os.path.join(self.libFolder, self.bakeObj, "bake.stl"))
        bpy.ops.view3d.snap_selected_to_cursor(use_offset=False)
        obj = context.active_object
        obj.name = self.bakeObj + "::instance"
        obj.phobostype = "link"
        obj["reference"] = self.bakeObj
        return {"FINISHED"}