#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import os
import bpy
import bpy.utils.previews
import phobos.utils.naming as nUtils
import phobos.utils.io as ioUtils
import phobos.utils.blender as bUtils
from phobos.phoboslog import log
from bpy.props import StringProperty, BoolProperty

# FIXME: the global variables get overwritten by the reload function
#        in phobos' __init__

model_data = {}
model_previews = {}
categories = set([])


def getModelListForEnumProperty(self, context):
    """Returns a list of (str, str, str) elements which contains the models
    contained in the currently selected model category.
    If there are no model categories (i.e. '-') return ('-', '-', '-').

    Args:
      context: 

    Returns:

    """
    category = context.window_manager.category
    if category == '-' or category == '':
        return [('-',) * 3]
    return sorted(model_previews[category].enum_items)


def getCategoriesForEnumProperty(self, context):
    """Returns a list of categories for an EnumProperty.
    
    The categories are based on the ``categories`` variable in the current namespace.
    
    If there are no categories return ('-', '-', '-').

    Args:
      context: 

    Returns:
      list: available category in the model library.

    """
    if not categories:
        return [('-',) * 3]
    return sorted([(item,) * 3 for item in categories])


def compileModelList():
    """TODO Missing documentation"""
    from bpy.props import EnumProperty
    from bpy.types import WindowManager

    # DOCU missing some docstring
    log("Compiling model list from local library...", "INFO")

    # clear old preview collections
    for previews in model_previews.values():
        bpy.utils.previews.remove(previews)
    model_previews.clear()
    model_data.clear()

    try:
        rootpath = bUtils.getPhobosPreferences().modelsfolder
    except KeyError:
        log('Can not create mechanism preview. Phobos not registered.', 'DEBUG')
        return

    if rootpath == '' or not os.path.exists(rootpath):
        log('Model library folder does not exist.')
        return

    # parse the model folder
    i = 0
    for category in os.listdir(rootpath):
        categorypath = os.path.join(rootpath, category)
        # skip all non folders
        if not os.path.isdir(categorypath):
            continue

        # initialise new dictionaries
        model_data[category] = {}
        newpreviewcollection = bpy.utils.previews.new()
        enum_items = []

        # parse category folder
        for modelname in os.listdir(categorypath):
            modelpath = os.path.join(categorypath, modelname)

            # check for valid blender savefile in the model folder
            if os.path.exists(os.path.join(modelpath, 'blender', modelname + '.blend')):
                model_data[category][modelname] = {'path': modelpath}

                # use existing thumbnail if available
                if os.path.exists(os.path.join(modelpath, 'thumbnails')):
                    previewpath = os.path.join(modelpath, 'thumbnails', modelname + '.png')
                    preview = newpreviewcollection.load(modelname, previewpath, 'IMAGE')
                # otherwise create one from the blend file
                else:
                    previewpath = os.path.join(modelpath, 'blender', modelname + '.blend')
                    preview = newpreviewcollection.load(modelname, previewpath, 'BLEND')
                log("Adding model to preview: " + previewpath, 'DEBUG')
                enum_items.append((modelname, modelname, "", preview.icon_id, i))
                i += 1
                categories.add(category)
        # save the category
        newpreviewcollection.enum_items = enum_items
        model_previews[category] = newpreviewcollection
        log("Finished parsing model folder. Imported {0} models.".format(i), 'INFO')


class UpdateModelLibraryOperator(bpy.types.Operator):
    """Update Model Library"""

    bl_idname = "phobos.update_model_library"
    bl_label = "Update Library"

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        compileModelList()
        return {'FINISHED'}


class ImportModelFromLibraryOperator(bpy.types.Operator):
    """TODO Missing documentation"""

    # DOCU add some docstring
    bl_idname = "phobos.import_model_from_library"
    bl_label = "Import Model"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    namespace = StringProperty(
        name="Namespace", default="", description="Namespace with which to wrap the imported model."
    )

    use_prefix = BoolProperty(
        name='Use Prefix',
        default=False,
        description="Import model with fixed prefixed instead of removable namespace.",
    )

    # as_reference = BoolProperty(
    #    name='Import reference'
    #    default=False,
    #    description="Import model as reference to original model instead of importing all elements.")

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        modelname = context.window_manager.modelpreview
        self.namespace = modelname
        # prevent duplicate names
        namespaces = nUtils.gatherNamespaces('__' if self.use_prefix else '::')
        if modelname in namespaces:
            i = 1
            self.namespace = modelname + '_' + str(i)
            while self.namespace in namespaces:
                i += 1
                self.namespace = modelname + '_' + str(i)
        return context.window_manager.invoke_props_dialog(self, width=500)

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """
        wm = context.window_manager
        # FIXME: the following is a hack to fix the problem mentioned at the top
        if not model_data:
            compileModelList()
        filepath = os.path.join(
            model_data[wm.category][wm.modelpreview]['path'], 'blender', wm.modelpreview + '.blend'
        )
        if ioUtils.importBlenderModel(filepath, self.namespace, self.use_prefix):
            return {'FINISHED'}
        else:
            log(
                "Model " + wm.modelpreview + " could not be loaded from library:"
                "No valid .blend file.",
                "ERROR",
            )
            return {'CANCELLED'}


def register():
    """TODO Missing documentation"""
    from bpy.types import WindowManager
    from bpy.props import StringProperty, EnumProperty, BoolProperty

    WindowManager.modelpreview : EnumProperty(items=getModelListForEnumProperty, name='Model')
    WindowManager.category : EnumProperty(items=getCategoriesForEnumProperty, name='Category')
    compileModelList()


def unregister():
    """TODO Missing documentation"""
    for previews in model_previews.values():
        bpy.utils.previews.remove(previews)
    model_previews.clear()
    model_data.clear()
