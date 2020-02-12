#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import bpy
from phobos.phoboslog import log
from phobos.utils.blender import createPreview


def exportPreview(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
      model(dict -- the generated robot model dictionary): The robot model to export
      path(str): The filepath to export the robot to. *WITH filename!*

    Returns:

    """
    log("Phobos Thumbnail export: Creating thumbnail in " + path, "INFO")
    visuals = []
    for linkname in model['links']:
        for visualname in model['links'][linkname]['visual']:
            visuals.append(bpy.data.objects[visualname])
    createPreview(visuals, path, model['name'])


# registering export functions of types with Phobos
entity_type_dict = {'thumbnails': {'export': exportPreview, 'extensions': ('png', 'jpg')}}
