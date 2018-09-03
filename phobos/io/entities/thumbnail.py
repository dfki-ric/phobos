#!/usr/bin/python
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
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
entity_type_dict = {'thumbnails': {'export': exportPreview,
                                   'extensions': ('png', 'jpg')}
                    }
