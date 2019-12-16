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
Contains the functions for the mechanism entity.
"""

import os
import bpy
import bpy.utils.previews
import phobos.defs as defs
import phobos.utils.blender as bUtils
from phobos.phoboslog import log


mechanismpreviewcollection = []


def getMechanismListForEnumProperty(self, context):
    """Returns a list of (str, str, str) elements which contains the mechanisms
    currently loaded. If there are none, returns ('-', '-', '-').

    Args:
      context: 

    Returns:

    """
    try:
        return sorted(mechanismpreviewcollection.enum_items)
    except AttributeError:
        log('No mechanism previews available. Check config folder.', 'ERROR')


def compileMechanismList():
    """TODO Missing documentation"""
    from bpy.types import WindowManager
    from bpy.props import EnumProperty

    # DOCU missing some docstring

    global mechanismpreviewcollection

    log("Compiling mechanism list from local library...", "INFO")

    try:
        phobosconfig = bUtils.getPhobosConfigPath()
    except KeyError:
        log('Can not create mechanism preview. Phobos not registered.', 'DEBUG')
        return

    imagefolderpath = os.path.join(phobosconfig, 'images', 'mechanisms')
    if imagefolderpath == '' or not os.path.exists(imagefolderpath):
        log('Visual mechanism representations could not be found.')
        return

    # read in mechanism thumbnails
    mechanismpreviewcollection = bpy.utils.previews.new()
    enum_items = []
    defaultimagepath = os.path.join(imagefolderpath, 'undefined.png')
    defaultpreview = mechanismpreviewcollection.load('undefined', defaultimagepath, 'IMAGE')

    i = 1
    for mechanism in defs.definitions['submechanisms']:
        size = len(defs.definitions['submechanisms'][mechanism]['joints']['spanningtree'])
        imagepath = os.path.join(imagefolderpath, mechanism + '.png')
        if not (os.path.exists(imagepath) and os.path.isfile(imagepath)):
            log("No preview found, using default.", 'DEBUG')
            enum_items.append(
                (mechanism, mechanism + ' [{0}] '.format(size), "", defaultpreview.icon_id, i)
            )
        else:
            log("Adding mechanism preview: " + imagepath, 'DEBUG')
            preview = mechanismpreviewcollection.load(mechanism, imagepath, 'IMAGE')
            enum_items.append(
                (mechanism, mechanism + '[ {0}]'.format(size), "", preview.icon_id, i)
            )
        i += 1
    mechanismpreviewcollection.enum_items = enum_items

    # reregister the enumproperty to ensure new items are displayed
    WindowManager.mechanismpreview = EnumProperty(
        items=getMechanismListForEnumProperty, name='Mechanism'
    )


def register():
    """TODO Missing documentation"""
    from bpy.types import WindowManager
    from bpy.props import EnumProperty

    WindowManager.mechanismpreview = EnumProperty(
        items=getMechanismListForEnumProperty, name='Mechanism'
    )
    compileMechanismList()


def unregister():
    """TODO Missing documentation"""
    bpy.utils.previews.remove(mechanismpreviewcollection)
