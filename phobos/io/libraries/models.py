#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.io.libraries.models.py
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators import/export

.. moduleauthor:: Kai von Szadowski, Simon Reichel

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

import os
import bpy
import bpy.utils.previews
import phobos.utils.naming as nUtils
# TODO ioUtils is not used
import phobos.utils.io as ioUtils
from phobos.phoboslog import log


def getModelListForEnumProperty(self, context):
    # DOCU missing some docstring
    category = context.window_manager.category
    return preview_collections[category].enum_items


def getCategoriesForEnumProperty(self, context):
    # DOCU missing some docstring
    return [(category,)*3 for category in sorted(preview_collections.keys())]


def compileModelList():
    # DOCU missing some docstring
    log("Compiling model list from local library...", "INFO", 'compileModelList')
    for previews in preview_collections.values():
        bpy.utils.previews.remove(previews)
    preview_collections.clear()
    model_data.clear()

    rootpath = bpy.context.user_preferences.addons["phobos"].preferences.modelsfolder
    i = 0
    if rootpath != '' and os.path.exists(rootpath):
        for category in os.listdir(rootpath):
            model_data[category] = {}
            newpreviewcollection = bpy.utils.previews.new()
            enum_items = []
            categorypath = os.path.join(rootpath, category)
            for modelname in os.listdir(categorypath):
                modelpath = os.path.join(categorypath, modelname)
                if os.path.exists(os.path.join(modelpath, 'blender', modelname+'.blend')):
                    model_data[category][modelname] = {'path': modelpath}
                    if os.path.exists(os.path.join(modelpath, 'thumbnails')):
                        preview = newpreviewcollection.load(modelname, os.path.join(modelpath, 'thumbnails', modelname+'.png'), 'IMAGE')
                        log("Adding model to path: " + os.path.join(modelpath, 'thumbnails', modelname+'.png'),
                            'DEBUG', 'compileModelList')
                    else:
                        preview = newpreviewcollection.load(modelname, os.path.join(modelpath, 'blender', modelname + '.blend'), 'BLEND')
                        log("Adding model to path: " + os.path.join(os.path.join(modelpath, 'blender', modelname + '.blend')),
                            'DEBUG', 'compileModelList')
                    enum_items.append((modelname, modelname, "", preview.icon_id, i))
                    i += 1
            newpreviewcollection.enum_items = enum_items
            preview_collections[category] = newpreviewcollection
    else:
        log('Model library folder does not exist.')

# TODO move this to beginning of file
model_data = {}
preview_collections = {}


class PhobosModelLibraryPanel(bpy.types.Panel):
    # DOCU add some docstring and update bl_idname
    bl_idname = "TOOLS_PT_PHOBOS_LOCALMODELS"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Phobos Models"
    bl_label = "Locally Model Library"

    def draw(self, context):
        layout = self.layout
        wm = context.window_manager
        layout.operator("phobos.update_model_library", icon="FILE_REFRESH")
        layout.prop(wm, 'category')
        layout.template_icon_view(wm, 'modelpreview', show_labels=True, scale=5.0)
        layout.prop(wm, 'modelpreview')
        layout.prop(wm, 'as_reference')
        layout.prop(wm, 'namespace')
        layout.label(text=wm.namespace+'::objectname' if wm.namespace != '' else 'no namespacing')
        layout.operator("phobos.import_model_from_library", icon="IMPORT")


class UpdateModelLibraryOperator(bpy.types.Operator):
    """Update Model Library"""
    bl_idname = "phobos.update_model_library"
    bl_label = "Update Library"

    def execute(self, context):
        compileModelList()
        return {'FINISHED'}


class ImportModelFromLibraryOperator(bpy.types.Operator):
    # DOCU add some docstring
    bl_idname = "phobos.import_model_from_library"
    bl_label = "Import Model"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        wm = context.window_manager
        filepath = os.path.join(model_data[wm.category][wm.modelpreview]['path'],
                                'blender', wm.modelpreview+'.blend')
        if (os.path.exists(filepath) and os.path.isfile(filepath) and
           filepath.endswith('.blend')):
            log("Importing model" + filepath, "INFO", 'ImportModelFromLibraryOperator')
            objects = []
            with bpy.data.libraries.load(filepath) as (data_from, data_to):
                for obj in data_from.objects:
                    objects.append({'name': obj})
            bpy.ops.wm.append(directory=filepath+"/Object/", files=objects)
            bpy.ops.view3d.view_selected(use_all_regions=False)
            if wm.namespace != '':
                for obj in bpy.context.selected_objects:
                    nUtils.addNamespace(obj, wm.namespace)
            return {'FINISHED'}
        else:
            log("Model " + wm.modelpreview + " could not be loaded from library: No valid .blend file.",
                "ERROR", 'ImportModelFromLibraryOperator')
            return {'CANCELLED'}


def register():
    from bpy.types import WindowManager
    from bpy.props import (
            StringProperty,
            EnumProperty,
            BoolProperty
            )
    WindowManager.modelpreview = EnumProperty(items=getModelListForEnumProperty, name='Model')
    # TODO delete me?
                                              #update=updateModelPreview)
    WindowManager.category = EnumProperty(items=getCategoriesForEnumProperty, name='Category')
    compileModelList()
    WindowManager.namespace = StringProperty(name='Name space')
    WindowManager.import_reference = BoolProperty(name='Import reference')
    WindowManager.as_reference = bpy.props.BoolProperty(name='As Reference')


def unregister():
    for previews in preview_collections.values():
        bpy.utils.previews.remove(previews)
    preview_collections.clear()
    model_data.clear()
