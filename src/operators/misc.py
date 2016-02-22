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
from phobos.logging import startLog, endLog, log
import phobos.defs as defs
import phobos.utils.selection as selectionUtils
import phobos.robotdictionary as robotdictionary
import phobos.validator as validator
import phobos.utils.general as generalUtils


current_robot_name = ''


def get_robot_names(scene, context):
    robot_names = [(root['modelname'],)*3 for root in selectionUtils.getRoots()]
    return robot_names


def get_pose_names(scene, context):
    poses = robotdictionary.get_poses(current_robot_name)
    pose_items = [(pose,)*3 for pose in poses]
    return pose_items


class SelectError(Operator):
    """Select an object with check errors"""
    bl_idname = "object.phobos_select_error"
    bl_label = "Select Erroneous Object"
    bl_options = {'REGISTER', 'UNDO'}

    errorObj = EnumProperty(
        name="Erroneous Objects",
        items=defs.generateCheckMessages,
        description="The objects containing errors")

    def execute(self, context):
        startLog(self)
        selectionUtils.selectByName(self.errorObj)
        for message in defs.checkMessages[self.errorObj]:
            log(message, 'INFO')
        endLog()

        return {'FINISHED'}


class CheckDict(Operator):
    """Check the robot dictionary"""
    bl_idname = "object.phobos_check_dict"
    bl_label = "Check Dictionary"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

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
    """Display mass of the selected objects in a pop-up window"""
    bl_idname = "object.phobos_calculate_mass"
    bl_label = "Calculate Mass"

    def execute(self, context):
        startLog(self)
        mass = generalUtils.calculateSum(context.selected_objects, 'mass')
        log("The calculated mass is: " + str(mass), "INFO")
        endLog()
        return {'FINISHED'}


class ShowDistanceOperator(Operator):
    """Show distance between two selected objects in world coordinates"""
    bl_idname = "object.phobos_show_distance"
    bl_label = "Show Distance"
    bl_options = {'REGISTER', 'UNDO'}

    distance = FloatProperty(
        name="Distance",
        default=0.0,
        subtype='DISTANCE',
        unit='LENGTH',
        precision=6,
        description="Distance between objects")

    distVector = FloatVectorProperty(
        name="Distance Vector",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="Distance vector between objects")

    def execute(self, context):
        startLog(self)
        self.distance, self.distVector = generalUtils.distance(context.selected_objects)
        log("distance: " + str(self.distance) + ", " + str(self.distVector), "INFO")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        return len(context.selected_objects) == 2

class StorePoseOperator(Operator):
    """Store the current pose of selected links in one of the scene's robots"""
    bl_idname = 'object.store_pose'
    bl_label = "Store Current Pose"
    bl_options = {'REGISTER', 'UNDO'}

    robot_name = EnumProperty(
        items=get_robot_names,
        name="Robot",
        description="Robot to store pose for"
    )

    pose_name = StringProperty(
        name="Pose Name",
        default="New Pose",
        description="Name of new pose"
    )

    def execute(self, context):
        robotdictionary.storePose(self.robot_name, self.pose_name)
        return {'FINISHED'}


class LoadPoseOperator(Operator):
    """Load a previously stored pose for one of the scene's robots"""
    bl_idname = 'object.load_pose'
    bl_label = "Load Pose"
    bl_options = {'REGISTER', 'UNDO'}

    robot_name = EnumProperty(
        items=get_robot_names,
        name="Robot Name",
        description="Robot to load a pose for"
    )

    pose_name = EnumProperty(
        items=get_pose_names,
        name="Pose Name",
        description="Name of pose to load"
    )

    def execute(self, context):
        global current_robot_name
        current_robot_name = self.robot_name
        robotdictionary.loadPose(self.robot_name, self.pose_name)
        return {'FINISHED'}
