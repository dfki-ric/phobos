#!/usr/bin/python
# coding=utf-8

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
__author__ = 'kavonszadkowski'

import os
import bpy
import bpy.utils.previews
from phobos.phoboslog import log


def getModelListForEnumProperty(self, context):
    category = context.window_manager.category
    return preview_collections[category].enum_items


def getCategoriesForEnumProperty(self, context):
    return [(category,)*3 for category in sorted(preview_collections.keys())]


def compileModelList():
    log("Compiling Model List...", "INFO")
    for previews in preview_collections.values():
        bpy.utils.previews.remove(previews)
    preview_collections.clear()
    model_data.clear()

    rootpath = bpy.context.user_preferences.addons["phobos"].preferences.modelsfolder
    i = 0
    for category in os.listdir(rootpath):
        model_data[category] = {}
        newpreviewcollection = bpy.utils.previews.new()
        enum_items = []
        categorypath = os.path.join(rootpath, category)
        for modelname in os.listdir(categorypath):
            modelpath = os.path.join(categorypath, modelname)
            if os.path.exists(os.path.join(modelpath, 'blender')):
                model_data[category][modelname] = {'path': modelpath}
                if os.path.exists(os.path.join(modelpath, 'thumbnails')):
                    preview = newpreviewcollection.load(modelname, os.path.join(modelpath, 'thumbnails', modelname+'.png'), 'IMAGE')
                    log("Adding model to path: "+os.path.join(modelpath, 'thumbnails', modelname+'.png'))
                else:
                    preview = newpreviewcollection.load(modelname, os.path.join(modelpath, 'blender', modelname+'.blend'), 'BLEND')
                    log("Adding model to path: "+os.path.join(os.path.join(modelpath, 'blender', modelname+'.blend')))
                enum_items.append((modelname, modelname, "", preview.icon_id, i))
                i += 1
        newpreviewcollection.enum_items = enum_items
        preview_collections[category] = newpreviewcollection


model_data = {}
preview_collections = {}


class PhobosModelLibraryPanel(bpy.types.Panel):
    bl_idname = "TOOLS_PT_PHOBOS_LOCALMODELS"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Phobos Models"
    bl_label = "Local Model Library"
    #bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
       layout = self.layout
       wm = context.window_manager
       layout.prop(wm, 'category')
       layout.template_icon_view(wm, 'previewlist')
       layout.prop(wm, 'previewlist')
       layout.separator()
       layout.label(text='Import')
       layout.operator("phobos.import_component", text="Import Component", icon="IMPORT")


class ImportComponent(bpy.types.Operator):
    bl_idname = "phobos.import_component"
    bl_label = ""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    #@classmethod
    #def poll(cls, context):
    #    return context is not None

    def execute(self, context):
        if self.filepath != '':
            log("Importing component" + self.filepath, "INFO", 'ImportComponentOperator')
            objects = []
            with bpy.data.libraries.load(self.filepath) as (data_from, data_to):
                for obj in data_from.objects:
                    objects.append({'name': obj})
            bpy.ops.wm.append(directory=self.filepath+"/Object/", files=objects)
            # with bpy.data.libraries.load(self.filepath) as (data_from, data_to):
            #     for attr in dir(data_to):
            #         print(attr)
            #         setattr(data_to, attr, getattr(data_from, attr))
            #with bpy.data.libraries.load(self.filepath) as (data_from, data_to):
            #    print(data_to)
            #    data_to.objects = data_from.objects
            #link object to current scene
            #for cat in ['armatures', 'materials', 'meshes', 'objects']:
                #for arm in data_to.armatures:
                #    bpy.data.armatures.
            #for obj in data_to.objects:
            #    bpy.context.scene.objects.link(obj)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}



def register():
    from bpy.types import WindowManager
    from bpy.props import (
            StringProperty,
            EnumProperty,
            )
    WindowManager.previewlist = EnumProperty(items=getModelListForEnumProperty,
                                             name='Model')
    WindowManager.category = EnumProperty(items=getCategoriesForEnumProperty,
                                          name='Category')
    compileModelList()



def unregister():
    for previews in preview_collections.values():
        bpy.utils.previews.remove(previews)
    preview_collections.clear()
    model_data.clear()
