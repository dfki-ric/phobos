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
Contains the Blender operators used to edit/add poses for Phobos models.
"""

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from bpy.props import StringProperty, EnumProperty
from bpy.types import Operator

from ..model import poses as poses
from ..utils import blender as bUtils
from ..utils import naming as nUtils
from ..utils import selection as sUtils


# this is partly redundant, but currently only needed here
def get_robot_names(scene, context):
    """

    Args:
      scene:
      context:

    Returns:

    """
    robot_names = [(nUtils.getModelName(root),) * 3 for root in (sUtils.getRootsOfSelection() if sUtils.getRootsOfSelection() else sUtils.getRoots())]
    return robot_names


# currently only needed here, no point in putting it in poses.py
def get_pose_names(current_robot_name):
    """

    Args:
      scene:
      context:

    Returns:

    """
    poselist = poses.getPoses(current_robot_name)
    return poselist


class StorePoseOperator(Operator):
    """Store the current pose of selected links in one of the scene's robots"""

    bl_idname = 'phobos.store_pose'
    bl_label = "Store Current Pose"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        """The operator is only active if there is at least one defined model root."""
        # A true model root has a 'model/name' property.
        for root in sUtils.getRoots():
            if 'model/name' in root:
                return True
        return False

    def updatePoseName(self, context):
        self.pose_name = self.pose_name.replace(" ", "_").replace(":", "_").replace("'", "_").replace('"', "_")

    robot_name : EnumProperty(
        items=get_robot_names, name="Robot", description="Robot to store pose for"
    )

    pose_name : StringProperty(
        name="Pose Name",
        default="",
        description="Name of new pose",
        update=updatePoseName
    )

    def invoke(self, context, event):
        """Opens a dialog to enter the pose name."""
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        root = sUtils.getObjectByProperty('model/name', self.robot_name)
        poses.storePose(root, self.pose_name)
        return {'FINISHED'}


class LoadPoseOperator(Operator):
    """Load a previously stored pose for one of the scene's robots"""

    bl_idname = 'phobos.load_pose'
    bl_label = "Load Pose"
    bl_options = {'REGISTER', 'UNDO'}

    def getPoseNames(self, context):
        return ((p,)*3 for p in ["No pose selected"] + get_pose_names(self.robot_name))

    robot_name : EnumProperty(
        items=get_robot_names, name="Robot Name", description="Robot to load a pose for"
    )

    pose_name : EnumProperty(
        items=getPoseNames, name="Pose Name", description="Name of pose to load"
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        if self.pose_name != "No pose selected":
            poses.loadPose(self.robot_name, self.pose_name)
        return {'FINISHED'}


class DeletePoseOperator(Operator):
    # [ToDo v2.1.0] make this a dialog like DissolveLink
    """Load a previously stored pose for one of the scene's robots"""

    bl_idname = 'phobos.delete_pose'
    bl_label = "Delete Pose"
    bl_options = {'REGISTER', 'UNDO'}

    def getPoseNames(self, context):
        return ((p,)*3 for p in ["No pose selected"] + get_pose_names(self.robot_name))

    robot_name : EnumProperty(
        items=get_robot_names, name="Robot Name", description="Robot to load a pose for"
    )

    pose_name : EnumProperty(
        items=getPoseNames, name="Pose Name", description="Name of pose to load"
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        if self.pose_name != "No pose selected":
            poses.deletePose(self.robot_name, self.pose_name)
        return {'FINISHED'}


# show robot model on 3dview
def draw_preview_callback(self):
    """Draws a 2D preview of the selected pose in the 3D View."""
    # Search for View_3d window
    area = bpy.context.area
    if area.type != 'VIEW_3D':
        # This can happen if the context changes, just stop drawing
        return

    modelsPosesColl = bUtils.getPhobosPreferences().models_poses
    activeModelPoseIndex = bpy.context.scene.active_ModelPose

    if not modelsPosesColl or activeModelPoseIndex >= len(bpy.data.images):
        return

    image_name = bpy.data.images[activeModelPoseIndex].name
    if image_name not in modelsPosesColl:
        return

    active_preview = modelsPosesColl[image_name]
    im = bpy.data.images[image_name]

    # Find the 3D view region
    region = None
    for r in area.regions:
        if r.type == 'WINDOW':
            region = r
            break
    if region is None:
        return

    view_width = region.width
    view_height = region.height

    # --- Modern GPU Drawing ---
    shader_img = gpu.shader.from_builtin('2D_IMAGE')
    shader_bg = gpu.shader.from_builtin('2D_UNIFORM_COLOR')
    
    # Calculate image placement
    tex_start_x = 50
    tex_end_x = view_width - 50
    tex_start_y = 50
    tex_end_y = view_height - 50
    
    if im.size[0] < view_width - 100:
        diff = int((view_width - im.size[0]) / 2)
        tex_start_x = diff
        tex_end_x = diff + im.size[0]
    if im.size[1] < view_height - 100:
        diff = int((view_height - im.size[1]) / 2)
        tex_start_y = diff
        tex_end_y = diff + im.size[1]

    # 1. Draw Background
    vertices_bg = (
        (0, 0), (view_width, 0),
        (0, view_height), (view_width, view_height)
    )
    indices_bg = ((0, 1, 2), (1, 2, 3))
    
    batch_bg = batch_for_shader(shader_bg, 'TRIS', {"pos": vertices_bg}, indices=indices_bg)
    shader_bg.bind()
    shader_bg.uniform_float("color", (0, 0, 0, 0.3))
    gpu.state.blend_set('ALPHA')
    batch_bg.draw(shader_bg)
    gpu.state.blend_set('NONE')

    # 2. Draw Image
    if im.bindcode is None:
        im.gl_load()
    
    shader_img.bind()
    shader_img.uniform_int("image", im.bindcode)
    
    vertices_img = (
        (tex_start_x, tex_start_y), (tex_end_x, tex_start_y),
        (tex_start_x, tex_end_y), (tex_end_x, tex_end_y)
    )
    tex_coords_img = ((0, 0), (1, 0), (0, 1), (1, 1))

    batch_img = batch_for_shader(
        shader_img, 'TRI_STRIP',
        {"pos": vertices_img, "texCoord": tex_coords_img}
    )
    batch_img.draw(shader_img)

    # 3. Draw Text
    font_id = 0
    font_size = 20
    gpu.text.size(font_id, font_size)
    gpu.text.position(font_id, tex_start_x, tex_end_y + 20, 0)
    gpu.text.color(font_id, 1.0, 1.0, 1.0, 1.0)
    gpu.text.draw(font_id, active_preview.label)


class ChangePreviewOperator(bpy.types.Operator):
    """TODO Missing documentation"""

    bl_idname = "scene.change_preview"
    bl_label = "Change the preview texture"

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        modelsPosesColl = bUtils.getPhobosPreferences().models_poses
        activeModelPoseIndex = bpy.context.scene.active_ModelPose
        if bpy.data.images[activeModelPoseIndex].name in modelsPosesColl.keys():
            activeModelPose = modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
            if activeModelPose.type != "robot_name":
                # show on view_3d
                root = None
                if context.view_layer.objects.active != None:
                    root = sUtils.getRoot(context.view_layer.objects.active)
                if (
                    not bpy.context.scene.preview_visible
                    and (bpy.data.images[activeModelPoseIndex].type == 'IMAGE')
                    and (
                        root is None
                        or not sUtils.isRoot(root)
                        or not (
                            modelsPosesColl[bpy.data.images[activeModelPoseIndex].name]
                            != root["model/name"]
                        )
                        or len(bpy.context.selected_objects) == 0
                    )
                ):
                    bpy.ops.view3d.draw_preview_operator()
                    bpy.context.scene.preview_visible = True

            else:
                activeModelPose.hide = not activeModelPose.hide
                if activeModelPose.hide:
                    activeModelPose.icon = "RIGHTARROW"
                else:
                    activeModelPose.icon = "DOWNARROW_HLT"
                for modelPose in modelsPosesColl:
                    if (modelPose.type != "robot_name") and (
                        modelPose.parent == activeModelPose.name
                    ):
                        modelPose.hide = activeModelPose.hide

        return {'FINISHED'}


class DrawPreviewOperator(bpy.types.Operator):
    """TODO Missing documentation"""

    bl_idname = "view3d.draw_preview_operator"
    bl_label = "Draw preview to the View3D"

    def modal(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        if event.type in {'LEFTMOUSE', 'RIGHTMOUSE', 'ESC'}:
            # TODO delete me?
            #            if bpy.context.scene.preview_visible:
            bpy.context.scene.preview_visible = False
            # TODO delete me?
            #           else:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        # the arguments we pass the the callback
        args = (self,)
        # Add the region OpenGL drawing callback
        # draw in view space with 'POST_VIEW' and 'PRE_VIEW'
        if hasattr(self, "_handle") and self._handle:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')

        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            draw_preview_callback, args, 'WINDOW', 'POST_PIXEL'
        )

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

classes = (
    StorePoseOperator,
    LoadPoseOperator,
    DeletePoseOperator,
    ChangePreviewOperator,
    DrawPreviewOperator,
)

def register():
    """TODO Missing documentation"""
    print("Registering operators.misc...")
    for classdef in classes:
        bpy.utils.register_class(classdef)


def unregister():
    """TODO Missing documentation"""
    print("Unregistering operators.misc...")
    for classdef in classes:
        bpy.utils.unregister_class(classdef)
