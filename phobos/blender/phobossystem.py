#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import os.path as path

import bpy


def getScriptsPath():
    """Returns the path for user-specific blender scripts for all major platforms

    Returns(str): scripts path

    Args:

    Returns:

    """
    scriptspath = path.normpath(
        path.expanduser(bpy.utils.user_resource(resource_type='SCRIPTS', path="addons"))
    )
    return scriptspath


def getConfigPath():
    """Returns the path for configuration data for all major platforms

    Returns(str): config path

    Args:

    Returns:

    """
    configpath = path.normpath(
        path.join(bpy.utils.user_resource(resource_type='SCRIPTS', path="addons"), "phobos", "data", "blender")
    )
    return configpath


def getBlenderConfigPath():
    """Returns the configuration path for user-specific blender data.

    Args:

    Returns:
      : str -- scripts path

    """
    configpath = path.normpath(
        path.expanduser(bpy.utils.user_resource(resource_type='CONFIG'))
    )
    return configpath
