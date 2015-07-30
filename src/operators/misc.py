#!/usr/bin/python

"""
.. module:: phobos.operators.misc
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators general purposes.

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
from bpy.props import BoolProperty, FloatProperty, FloatVectorProperty, EnumProperty, StringProperty
from bpy.types import Operator
from phobos.logging import adjustLevel, startLog, endLog, log
import phobos.defs as defs
import phobos.utils.selection as selectionUtils
import phobos.robotdictionary as robotdictionary
import phobos.validator as validator
import phobos.utils.general as generalUtils
import phobos.utils.blender as blenderUtils


class SelectError(Operator):
    """SelectErrorOperator

    """
    bl_idname = "object.phobos_select_error"
    bl_label = "Selects an object with check errors"
    bl_options = {'REGISTER', 'UNDO'}

    errorObj = EnumProperty(
        name="Error containing objects",
        items=defs.generateCheckMessages,
        description="The objects containing errors.")

    def execute(self, context):
        startLog(self)
        selectionUtils.selectByName(self.errorObj)
        for message in defs.checkMessages[self.errorObj]:
            log(message, 'INFO')
        endLog()

        return {'FINISHED'}


class CheckDict(Operator):
    """CheckDictOperator

    """

    bl_idname = "object.phobos_check_dict"
    bl_label = "Checks the robotdictionary"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        """Executes the operator and unifies the selected objects meshes

        :param context: The blender context to work with
        :return: Blender result
        """
        startLog(self)
        messages = {}
        dic = robotdictionary.buildRobotDictionary()
        validator.check_dict(dic, defs.dictConstraints, messages)
        defs.checkMessages = messages if len(list(messages.keys())) > 0 else {"NoObject": []}
        for entry in messages:
            log("Errors in object " + entry + ":", 'INFO')
            for error in messages[entry]:
                log(error, 'INFO')
        endLog()
        return {'FINISHED'}


class CalculateMassOperator(Operator):
    """CalculateMassOperator

    """
    bl_idname = "object.phobos_calculate_mass"
    bl_label = "Display mass of the selected objects in a pop-up window."

    def execute(self, context):
        startLog(self)
        mass = generalUtils.calculateSum(bpy.context.selected_objects, 'mass')
        log("The calculated mass is: " + str(mass), "INFO")
        endLog()
        return {'FINISHED'}


class ShowDistanceOperator(Operator):
    """ShowDistanceOperator

    """
    bl_idname = "object.phobos_show_distance"
    bl_label = "Shows distance between two selected objects in world coordinates."
    bl_options = {'REGISTER', 'UNDO'}

    distance = FloatProperty(
        name="distance",
        default=0.0,
        subtype='DISTANCE',
        unit='LENGTH',
        precision=6,
        description="distance between objects")

    distVector = FloatVectorProperty(
        name="distanceVector",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="distance between objects")

    def execute(self, context):
        startLog(self)
        self.distance, self.distVector = generalUtils.distance(bpy.context.selected_objects)
        log("distance: " + str(self.distance) + ", " + str(self.distVector), "INFO")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        return len(context.selected_objects) == 2


class SetLogSettings(Operator):
    """Adjust Logging Settings for phobos

    """
    bl_idname = 'object.phobos_adjust_logger'
    bl_label = "Change the detail of the phobos logger"
    bl_options = {'REGISTER', 'UNDO'}

    isEnabled = BoolProperty(
        name="Enable logging",
        default=True,
        description="Enable log messages (INFOS will still appear)"
    )

    errors = BoolProperty(
        name="Show Errors",
        default=True,
        description="Show errors in log"
    )

    warnings = BoolProperty(
        name="Show Warnings",
        default=True,
        description="Show warnings in log"
    )

    def execute(self, context):
        adjustLevel("ALL", self.isEnabled)
        adjustLevel("ERROR", self.errors)
        adjustLevel("WARNING", self.warnings)
        return {'FINISHED'}


class StorePoseOperator(Operator):
    """
    """
    bl_idname = 'object.store_pose'
    bl_label = "Store the robot's current pose"
    bl_options = {'REGISTER', 'UNDO'}

    pose_name = StringProperty(
        name="Pose Name",
        default="New Pose",
        description="Name of new pose"
    )

    def execute(self, context):
        robotdictionary.storePose(self.pose_name)
        return {'FINISHED'}


class LoadPoseOperator(Operator):
    """
    """
    bl_idname = 'object.load_pose'
    bl_label = "Load a pose for the robot"
    bl_options = {'REGISTER', 'UNDO'}

    pose_name = StringProperty(
        name="Pose Name",
        default="New Pose",
        description="Name of pose to load"
    )

    def execute(self, context):
        blenderUtils.loadPose(self.pose_name)
        return {'FINISHED'}
