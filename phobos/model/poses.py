#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski

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

File meshes.py

Created on 13 Feb 2014
"""

import os
import bpy
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
from phobos.phoboslog import log
from phobos.utils.io import securepath


def deriveObjectPose(obj):
    """Derives a pose of link, visual or collision object.

    The transformations of the object are calculated according to
    phobos.utils.edititing.getCombinedTransform.

    The returned dictionary contains this information:
        *rawmatrix*: mathutils.Matrix
        *matrix*: list representation (list of lists) of mathutils.Matrix
        *translation*: list (according to mathutils.Matrix.to_translation)
        *rotation_euler*: list (according to mathutils.Matrix.to_euler)
        *rotation_quaternion*: list (according to mathutils.Matrix.to_quaternion)

    :param obj: blender object to derive the pose from
    :type obj: bpy.types.Object

    :return: pose information of the object
    :rtype: dict

    .. seealso phobos.utils.editing.getCombinedTransform
    """
    effectiveparent = sUtils.getEffectiveParent(obj)
    matrix = eUtils.getCombinedTransform(obj, effectiveparent)

    pose = {'rawmatrix': matrix,
            'matrix': [list(vector) for vector in list(matrix)],
            'translation': list(matrix.to_translation()),
            'rotation_euler': list(matrix.to_euler()),
            'rotation_quaternion': list(matrix.to_quaternion())
            }
    return pose

# TODO delete me?
#def bakeAllPoses(objlist, modelname, posename="", savetosubfolder=True):


def bakeModel(objlist, modelname, posename="", decimate_type='COLLAPSE', decimate_parameter=0.1):
    """This function gets a list of objects and creates a single, simplified mesh from it and exports it to .stl.

    Args:
      objlist(list: list): The list of blender objects to join and export as simplified stl file.
      modelname(str): The new models name and filename.
      posename:  (Default value = "")
      decimate_type:  (Default value = 'COLLAPSE')
      decimate_parameter:  (Default value = 0.1)

    Returns:

    """
    if bpy.data.window_managers[0].phobosexportsettings.relativePath:
        # CHECK careful with path consistency (Windows)
        outpath = securepath(os.path.expanduser(
            os.path.join(bpy.path.abspath("//"),
                         bpy.data.window_managers[0].phobosexportsettings.path)))
    else:
        # CHECK careful with path consistency (Windows)
        outpath = securepath(os.path.expanduser(
            bpy.data.window_managers[0].phobosexportsettings.path))

    # TODO delete me?
    #bake_outpath = securepath(os.path.join(outpath, modelname) if savetosubfolder else outpath)
    bake_outpath = outpath

    if bpy.data.window_managers[0].phobosexportsettings.structureExport:
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
        createPreview(visuals, export_path=bake_outpath, modelname=modelname, previewfile=previewfile)

        obj.select = True

        bpy.ops.object.delete()
        log("Done baking...", "INFO")

    else:
        log("No visuals to bake!", "WARNING")

    # TODO better use logging, right?
    print("Done baking...")
