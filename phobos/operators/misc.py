#!/usr/bin/python
# coding=utf-8

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
import blf
import bgl
from bpy.props import BoolProperty, FloatProperty, FloatVectorProperty, EnumProperty, StringProperty
from bpy.types import Operator
from phobos.logging import startLog, endLog, log
import phobos.defs as defs
import phobos.utils.selection as sUtils
import phobos.utils.general as gUtils
import phobos.model.robot as robot
import phobos.validator as validator

# FIXME: this is ugly
current_robot_name = ''

# FIXME: this should be treated in selection utils
def get_robot_names(scene, context):
    robot_names = [(root['modelname'],)*3 for root in sUtils.getRoots()]
    return robot_names

# FIXME: this should not go here either
def get_pose_names(scene, context):
    poses = model.getPoses(current_robot_name)
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
        sUtils.selectByName(self.errorObj)
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
        root = sUtils.getRoot(context.selected_objects[0])
        model, objectlist = robot.buildModelDictionary(root)
        validator.check_dict(model, defs.dictConstraints, messages)
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
        mass = gUtils.calculateSum(context.selected_objects, 'mass')
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
        self.distance, self.distVector = gUtils.distance(context.selected_objects)
        log("distance: " + str(self.distance) + ", " + str(self.distVector), "INFO")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        return len(context.selected_objects) == 2


class StorePoseOperator2(Operator):
    """Store the current pose of selected links in one of the scene's robots"""
    bl_idname = 'object.store_pose2'
    bl_label = "Store Current Pose"

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

    @classmethod
    def poll(self, context):
        result = False
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        if (len(bpy.context.selected_objects) > 0) and \
           (context.scene.objects.active != None) and \
           (sUtils.isModelRoot(sUtils.getRoot(context.scene.objects.active))) and \
           (bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys()):
            result = True
        return result

    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_props_dialog(self,width=300,height=100)

    def draw(self, context):
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        row = self.layout
        if modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].type == 'robot_pose':
            self.pose_name = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].label
        else:
            self.pose_name = "New Pose"
        row.prop(self, "pose_name")

    def execute(self, context):
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        robot_name = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].robot_name
        robot.storePose(robot_name, self.pose_name)
        bpy.ops.scene.reload_models_and_poses_operator()
        newPose = modelsPosesColl.add()
        newPose.parent = robot_name
        newPose.name = robot_name + '_' + self.pose_name
        newPose.label = self.pose_name
        #newPose.path = pose["robotpath"]
        newPose.type = "robot_pose"
        newPose.robot_name = robot_name
        newPose.icon = "X_VEC"
        return {'FINISHED'}


class LoadPoseOperator2(Operator):
    """Load a previously stored pose for one of the scene's robots"""
    bl_idname = 'object.load_pose2'
    bl_label = "Load Selected Pose"

    @classmethod
    def poll(self, context):
        result = False
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        root = sUtils.getRoot(context.scene.objects.active)
        if (len(bpy.context.selected_objects) > 0) and \
           (context.scene.objects.active != None) and \
           (sUtils.isModelRoot(root)) and \
           (bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys()) and \
           (modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].robot_name == root["modelname"]) and \
           (modelsPosesColl[bpy.data.images[activeModelPoseIndex].name].type == 'robot_pose'):
            result = True
        return result

    def execute(self, context):
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        modelPose = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
        root = sUtils.getRoot(context.scene.objects.active)
        bpy.context.scene.objects.active = root
        robot.loadPose(modelPose.robot_name, modelPose.label)
        return {'FINISHED'}


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
        robot.storePose(self.robot_name, self.pose_name)
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
        robot.loadPose(self.robot_name, self.pose_name)
        return {'FINISHED'}


# show robot model on 3dview
def draw_preview_callback(self):

    # Search for View_3d window
    area = None
    if bpy.context.area.type != 'VIEW_3D':
        return bpy.context.area
    else:
        for oWindow in bpy.context.window_manager.windows:
            oScreen = oWindow.screen
            for oArea in oScreen.areas:
                if oArea.type == 'VIEW_3D':
                    area = oArea

    modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
    activeModelPoseIndex = bpy.context.scene.active_ModelPose

    if (len(modelsPosesColl)>0) and area:

        # Draw a textured quad
        area_widths  = [region.width for region in bpy.context.area.regions if region.type=='WINDOW']
        area_heights = [region.height for region in bpy.context.area.regions if region.type=='WINDOW']
        if (len(area_widths)>0) and (len(area_heights)>0):

            active_preview = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
            im = bpy.data.images[activeModelPoseIndex]

            view_width = area_widths[0]
            view_height = area_heights[0]
            tex_start_x = 50
            tex_end_x = view_width-50
            tex_start_y = 50
            tex_end_y = view_height-50
            if im.size[0] < view_width:
                diff = int((view_width-im.size[0])/2)
                tex_start_x = diff
                tex_end_x   = diff+im.size[0]
            if im.size[1] < view_height:
                diff = int((view_height-im.size[1])/2)
                tex_start_y = diff
                tex_end_y   = diff+im.size[1]

            # Draw information
            font_id = 0  # XXX, need to find out how best to get this.
            blf.position(font_id, tex_start_x, tex_end_y + 20, 0)
            blf.size(font_id, 20, 72)
            blf.draw(font_id, active_preview.label)

            tex = im.bindcode
            bgl.glEnable(bgl.GL_TEXTURE_2D)
            # if using blender 2.77 change tex to tex[0]
            bgl.glBindTexture(bgl.GL_TEXTURE_2D, tex)

            # Background
            bgl.glEnable(bgl.GL_BLEND)

            bgl.glBegin(bgl.GL_QUADS)
            bgl.glColor4f(0, 0, 0, 0.3)
            bgl.glVertex2i(0, 0)
            bgl.glVertex2i(0, view_height)
            bgl.glVertex2i(view_width, view_height)
            bgl.glVertex2i(view_width, 0)

            # Draw Image
            bgl.glColor4f(1, 1, 1, 1)
            bgl.glTexCoord2f(0, 0)
            bgl.glVertex2i(int(tex_start_x), int(tex_start_y))
            bgl.glTexCoord2f(0, 1)
            bgl.glVertex2i(int(tex_start_x), int(tex_end_y))
            bgl.glTexCoord2f(1, 1)
            bgl.glVertex2i(int(tex_end_x), int(tex_end_y))
            bgl.glTexCoord2f(1, 0)
            bgl.glVertex2i(int(tex_end_x), int(tex_start_y))
            bgl.glEnd()

            # restore opengl defaults
            bgl.glColor4f(0.0, 0.0, 0.0, 1.0)
            bgl.glDisable(bgl.GL_QUADS)
            bgl.glDisable(bgl.GL_BLEND)
            bgl.glDisable(bgl.GL_TEXTURE_2D)


class ChangePreviewOperator(bpy.types.Operator):
    bl_idname = "scene.change_preview"
    bl_label = "Change the preview texture"

    def execute(self, context):
        modelsPosesColl = bpy.context.user_preferences.addons["phobos"].preferences.models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        if bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys():
            activeModelPose = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
            if activeModelPose.type != "robot_name":
                # show on view_3d
                root = None
                if context.scene.objects.active != None:
                    root = sUtils.getRoot(context.scene.objects.active)
                if not bpy.context.scene.preview_visible and \
                        (bpy.data.images[activeModelPoseIndex].type == 'IMAGE') and \
                        (root == None or not sUtils.isModelRoot(root) or not (modelsPosesColl[bpy.data.images[activeModelPoseIndex].name] != root["modelname"]) or len(bpy.context.selected_objects) == 0):
                    bpy.ops.view3d.draw_preview_operator()
                    bpy.context.scene.preview_visible = True

            else:
                activeModelPose.hide = not activeModelPose.hide
                if activeModelPose.hide:
                    activeModelPose.icon = "RIGHTARROW"
                else:
                    activeModelPose.icon = "DOWNARROW_HLT"
                for modelPose in modelsPosesColl:
                    if (modelPose.type != "robot_name") and (modelPose.parent == activeModelPose.name):
                        modelPose.hide = activeModelPose.hide

        return {'FINISHED'}


class DrawPreviewOperator(bpy.types.Operator):
    bl_idname = "view3d.draw_preview_operator"
    bl_label = "Draw preview to the View3D"

    def modal(self, context, event):
        if event.type in {'LEFTMOUSE','RIGHTMOUSE','ESC'}:
#            if bpy.context.scene.preview_visible:
            bpy.context.scene.preview_visible = False
 #           else:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        # the arguments we pass the the callback
        args = (self,)
        # Add the region OpenGL drawing callback
        # draw in view space with 'POST_VIEW' and 'PRE_VIEW'
        if hasattr(self,"_handle") and self._handle:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')

        self._handle = bpy.types.SpaceView3D.draw_handler_add(draw_preview_callback, args, 'WINDOW', 'POST_PIXEL')

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}


