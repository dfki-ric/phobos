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
Contains all functions to model poses in Blender.
"""

import json
import os

import bpy
import numpy as np

from ..phoboslog import log
from ..utils import blender as bUtils
from ..utils import selection as sUtils
from ..utils.io import securepath, getExportPath


def bakeModel(objlist, modelname, posename="", decimate_type='COLLAPSE', decimate_parameter=0.1):
    """This function gets a list of objects and creates a single, simplified mesh from it and exports it to .stl.

    Args:
      objlist(list: list): The list of blender objects to join and export as simplified stl file.
      modelname(str): The new models name and filename.
      posename: (Default value = "")
      decimate_type: (Default value = 'COLLAPSE')
      decimate_parameter: (Default value = 0.1)
      objlist: 

    Returns:

    """
    if not os.path.isabs(getExportPath()):
        # CHECK careful with path consistency (Windows)
        outpath = securepath(
            os.path.expanduser(
                os.path.join(bpy.path.abspath("//"), getExportPath())
            )
        )
    else:
        # CHECK careful with path consistency (Windows)
        outpath = securepath(os.path.expanduser(getExportPath()))

    # TODO delete me?
    # bake_outpath = securepath(os.path.join(outpath, modelname) if savetosubfolder else outpath)
    bake_outpath = outpath

    if bpy.context.scene.phobosexportsettings.structureExport:
        securepath(os.path.join(bake_outpath, 'bakes'))
        bake_outpath = os.path.join(bake_outpath, 'bakes/')

    export_name = modelname + '_' + posename

    visuals = [o for o in objlist if ("phobostype" in o and o.phobostype == "visual")]
    if len(visuals) > 0:

        log("Baking model to " + bake_outpath, "INFO")
        sUtils.selectObjects(visuals, active=0)
        log("Copying objects for joining...", "INFO")
        bpy.ops.object.duplicate(linked=False, mode='TRANSLATION')
        log("Joining...", "INFO")
        bpy.ops.object.join()
        obj = bpy.context.active_object
        log("Deleting vertices...", "INFO")
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.select_all(action='TOGGLE')
        bpy.ops.mesh.select_all(action='TOGGLE')
        bpy.ops.mesh.remove_doubles()
        bpy.ops.object.editmode_toggle()
        log("Adding modifier...", "INFO")

        bpy.ops.object.modifier_add(type='DECIMATE')
        bpy.context.object.modifiers["Decimate"].decimate_type = decimate_type
        if decimate_type == 'COLLAPSE':
            bpy.context.object.modifiers["Decimate"].ratio = decimate_parameter
        elif decimate_type == 'UNSUBDIV':
            bpy.context.object.modifiers["Decimate"].iterations = decimate_parameter
        elif decimate_type == 'DISSOLVE':
            bpy.context.object.modifiers["Decimate"].angle_limit = decimate_parameter

        log("Applying modifier...", "INFO")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Decimate")
        obj.name = export_name + ".obj"

        # TODO use_selection might cause bugs, depending on Blender version
        bpy.ops.export_scene.obj(filepath=os.path.join(bake_outpath, obj.name), use_selection=True)

        obj.hide_render = True
        previewfile = export_name
        bUtils.createPreview(
            visuals, export_path=bake_outpath, modelname=modelname, previewfile=previewfile
        )

        obj.select_set(True)

        bpy.ops.object.delete()
        log("Done baking...", "INFO")

    else:
        log("No visuals to bake!", "WARNING")


def storePose(root, posename):
    """Stores the current pose of all of a model's selected joints.
    
    Existing poses of the same name will be overwritten.

    Args:
      root(bpy_types.Object): root of the model the pose belongs to
      posename(str): name the pose will be stored under

    Returns:
      : Nothing.

    """
    if not posename:
        log("No pose name given", "WARN")
        return
    if root:
        links = sUtils.getChildren(root, ('link',), True, False)
        sUtils.selectObjects([root] + links, clear=True, active=0)
        bpy.ops.object.mode_set(mode='POSE')
        for link in (
            link
            for link in links
            if 'joint/type' in link and link['joint/type'] not in ['fixed', 'floating']
        ):
            if link["joint/type"] == "prismatic":
                link["pose/"+posename] = np.round(link.pose.bones['Bone'].location.y, decimals=6)
            else:
                link.pose.bones['Bone'].rotation_mode = 'XYZ'
                link["pose/"+posename] = np.round(link.pose.bones['Bone'].rotation_euler.y, decimals=6)
            if "joint/limits/lower" in link:
                link["pose/"+posename] = max(link["pose/"+posename], link["joint/limits/lower"])
            if "joint/limits/upper" in link:
                link["pose/"+posename] = min(link["pose/"+posename], link["joint/limits/upper"])
        bpy.ops.object.mode_set(mode='OBJECT')
    else:
        log("No model root provided to store the pose for", "WARNING")


def loadPose(modelname, posename):
    """Load and apply a robot's stored pose.

    Args:
      modelname(str): the model's name
      posename(str): the name the pose is stored under

    Returns:

    """
    if not modelname:
        log("No model name given", "WARN")
        return
    if not posename:
        log("No pose name given", "WARN")
        return
    root = sUtils.getModelRoot(modelname)
    if not root:
        log("No model found with the name "+modelname, "ERROR")
    links = sUtils.getChildren(root, ('link',), True, False)
    sUtils.selectObjects([root] + links, clear=True, active=0)
    found = False
    try:
        bpy.ops.object.mode_set(mode='POSE')
        for link in (
                link
                for link in links
                if 'joint/type' in link and link['joint/type'] not in ['fixed', 'floating']
        ):
            if "pose/" + posename in link:
                found = True
                if link["joint/type"] == "prismatic":
                    link.pose.bones['Bone'].location.y = link["pose/" + posename]
                else:
                    link.pose.bones['Bone'].rotation_mode = "XYZ"
                    link.pose.bones['Bone'].rotation_euler.y = link["pose/"+posename]
        bpy.ops.object.mode_set(mode='OBJECT')
    finally:
        # restore previous mode
        bpy.ops.object.mode_set(mode='OBJECT')

    if not found:
        log(f"No pose with name {posename} in model {modelname}", "ERROR")


def deletePose(modelname, posename):
    """Load and apply a robot's stored pose.

    Args:
      modelname(str): the model's name
      posename(str): the name the pose is stored under

    Returns:

    """
    if not modelname:
        log("No model name given", "WARN")
        return
    if not posename:
        log("No pose name given", "WARN")
        return
    root = sUtils.getModelRoot(modelname)
    if not root:
        log("No model found with the name "+modelname, "ERROR")
    links = sUtils.getChildren(root, ('link',), True, False)
    sUtils.selectObjects([root] + links, clear=True, active=0)
    found = False
    for link in (
            link
            for link in links
            if 'joint/type' in link and link['joint/type'] not in ['fixed', 'floating']
    ):
        if "pose/" + posename in link:
            found = True
            link.pop("pose/" + posename)

    if not found:
        log(f"No pose with name {posename} in model {modelname}", "ERROR")
    else:
        log(f"Pose {posename} has been deleted", "INFO")


def getPoses(modelname):
    """Get the names of the poses that have been stored for a robot.

    Args:
      modelname: The model's name.

    Returns:
      : A list containing the poses' names.

    """
    if not modelname:
        log("No model name given", "WARN")
        return
    root = sUtils.getModelRoot(modelname)
    if not root:
        log("No model found with the name " + modelname, "ERROR")
        return
    links = sUtils.getChildren(root, ('link',), True, False)
    sUtils.selectObjects([root] + links, clear=True, active=0)
    poses = set()
    for link in (
            link
            for link in links
            if 'joint/type' in link and link['joint/type'] not in ['fixed', 'floating']
    ):
        for k in link.keys():
            if k.startswith("pose/"):
                poses.add(k[5:])
    return list(poses)
