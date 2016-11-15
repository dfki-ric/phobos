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
from phobos.logging import log
from phobos.utils.io import securepath


def deriveObjectPose(obj):
    """This function derives a pose of link, visual or collision object.

    :param obj: The blender object to derive the pose from.
    :type obj: bpy_types.Object
    :return: dict

    """
    matrix = obj.matrix_local
    effectiveparent = sUtils.getEffectiveParent(obj)
    parent = obj.parent
    while parent != effectiveparent and parent is not None:
        matrix = parent.matrix_local * matrix
        parent = parent.parent
    pose = {'rawmatrix': matrix,
            'matrix': [list(vector) for vector in list(matrix)],
            'translation': list(matrix.to_translation()),
            'rotation_euler': list(matrix.to_euler()),
            'rotation_quaternion': list(matrix.to_quaternion())
            }
    return pose


def createPreview(objects, export_path, modelname, previewfile, render_resolution=256):
    """This function creates a thumbnail of the given objects.

    :param obj: List of objects for the thumbnail.
    :type obj: list
    :param Resolution used for the render.
    :type int

    """
    # thumbnail

    # check/create camera
    # TODO create camera if there is no
    cam_ob = bpy.context.scene.camera
    # cam = bpy.data.cameras.new("Camera")
    # delete_cam = False
    if not cam_ob:
        log("No Camera found! Can not create thumbnail", "WARNING", __name__ + ".bakeModel")
        return
        #cam_ob = bpy.data.objects.new("Camera", cam)
        #bpy.context.scene.objects.link(cam_ob)
        #delete_cam = True
    #bpy.context.scene.camera = cam_ob

    log("Creating thumbnail of model: "+modelname, "INFO",__name__+".bakeModel")
    # hide all other objects from rendering
    for ob in bpy.data.objects:
        if not (ob in objects) and not(ob.type == 'LAMP'):
            ob.hide_render = True
            ob.hide = True

    # set render settings
#    bpy.context.scene.render.resolution_x = render_resolution
#    bpy.context.scene.render.resolution_y = render_resolution
#    bpy.context.scene.render.resolution_percentage = 100
    # render
    #bpy.ops.render.render(use_viewport=True)
    bpy.ops.render.opengl(view_context=True)
    # save image
    bpy.context.scene.render.image_settings.file_format = 'PNG'
#    bpy.data.images['Render Result'].file_format = bpy.context.scene.render.image_settings.file_format

    #print(bpy.data.images['Render Result'].file_format)
    log("saving preview to: "+os.path.join(export_path,previewfile+'.png'), "INFO",__name__+".bakeModel")

    bpy.data.images['Render Result'].save_render(os.path.join(export_path,previewfile+'.png'))


    # make all objects visible again
    for ob in bpy.data.objects:
        ob.hide_render = False
        ob.hide = False

    # delete camera if needed
    #if delete_cam:
    #    bpy.ops.object.select_all(action='DESELECT')
    #    cam_ob.select = True
    #    bpy.ops.object.delete()

#def bakeAllPoses(objlist, modelname, posename="", savetosubfolder=True):



def bakeModel(objlist, modelname, posename="", decimate_type='COLLAPSE', decimate_parameter=0.1):
    """This function gets a list of objects and creates a single, simplified mesh from it and exports it to .stl.

    :param objlist: The list of blender objects to join and export as simplified stl file.
    :type objlist: list
    :param modelname: The new models name and filename.
    :type modelname: str

    """
    if bpy.data.worlds[0].phobosexportsettings.relativePath:
        outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].phobosexportsettings.path)))
    else:
        outpath = securepath(os.path.expanduser(bpy.data.worlds[0].phobosexportsettings.path))

    #bake_outpath = securepath(os.path.join(outpath, modelname) if savetosubfolder else outpath)
    bake_outpath = outpath

    if bpy.data.worlds[0].phobosexportsettings.structureExport:
        securepath(os.path.join(bake_outpath, 'bakes'))
        bake_outpath = os.path.join(bake_outpath, 'bakes/')

    export_name = modelname+ '_' + posename

    visuals = [o for o in objlist if ("phobostype" in o and o.phobostype == "visual")]
    if len(visuals) > 0:

        log("Baking model to " + bake_outpath, "INFO",__name__+".bakeModel")
        sUtils.selectObjects(visuals, active=0)
        log("Copying objects for joining...", "INFO",__name__+".bakeModel")
        bpy.ops.object.duplicate(linked=False, mode='TRANSLATION')
        log("Joining...", "INFO",__name__+".bakeModel")
        bpy.ops.object.join()
        obj = bpy.context.active_object
        log("Deleting vertices...", "INFO",__name__+".bakeModel")
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.select_all(action='TOGGLE')
        bpy.ops.mesh.select_all(action='TOGGLE')
        bpy.ops.mesh.remove_doubles()
        bpy.ops.object.editmode_toggle()
        log("Adding modifier...", "INFO",__name__+".bakeModel")

        bpy.ops.object.modifier_add(type='DECIMATE')
        bpy.context.object.modifiers["Decimate"].decimate_type = decimate_type
        if decimate_type == 'COLLAPSE':
            bpy.context.object.modifiers["Decimate"].ratio = decimate_parameter
        elif decimate_type == 'UNSUBDIV':
            bpy.context.object.modifiers["Decimate"].iterations = decimate_parameter
        elif decimate_type == 'DISSOLVE':
            bpy.context.object.modifiers["Decimate"].angle_limit = decimate_parameter


        log("Applying modifier...", "INFO",__name__+".bakeModel")
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Decimate")
        obj.name = export_name + ".obj"

        bpy.ops.export_scene.obj(filepath=os.path.join(bake_outpath, obj.name),use_selection=True)

        obj.hide_render = True
        previewfile = export_name
        createPreview(visuals, export_path=bake_outpath, modelname=modelname, previewfile=previewfile)

        obj.select = True

        bpy.ops.object.delete()
        log("Done baking...", "INFO")

    else:
        log("No visuals to bake!","WARNING")

    print("Done baking...")
