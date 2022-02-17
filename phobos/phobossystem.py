#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import sys
import os.path as path
import bpy

# compatible blender version
blenderversion = '2.91'


def getScriptsPath():
    """Returns the path for user-specific blender scripts for all major platforms
    
    Returns(str): scripts path

    Args:

    Returns:

    """
    if sys.platform == 'linux':
        scriptspath = path.normpath(
            path.expanduser('~/.config/blender/{0}/scripts'.format(blenderversion))
        )
    elif sys.platform == 'darwin':
        scriptspath = path.normpath(
            path.expanduser(
                '~/Library/Application Support/Blender/{0}/scripts'.format(blenderversion)
            )
        )
    elif sys.platform == 'win32':
        scriptspath = path.normpath(
            path.expanduser(
                '~/AppData/Roaming/Blender Foundation/Blender/{0}/scripts'.format(blenderversion)
            )
        )
    else:
        scriptspath = 'ERROR: {0} not supported,'.format(sys.platform)
    return scriptspath


def getConfigPath():
    """Returns the path for configuration data for all major platforms
    
    Returns(str): config path

    Args:

    Returns:

    """
    configpath = path.normpath(path.join(bpy.utils.user_resource(resource_type='SCRIPTS', path="addons"), "phobos", "config"))
    return configpath


def getBlenderConfigPath():
    """Returns the configuration path for user-specific blender data.

    Args:

    Returns:
      : str -- scripts path

    """
    if sys.platform == 'linux':
        scriptspath = path.normpath(
            path.expanduser('~/.config/blender/{0}/config'.format(blenderversion))
        )
    elif sys.platform == 'darwin':
        scriptspath = path.normpath(
            path.expanduser(
                '~/Library/Application Support/Blender/{0}/config'.format(blenderversion)
            )
        )
    elif sys.platform == 'win32':
        scriptspath = path.normpath(
            path.expanduser(
                '~/AppData/Roaming/Blender Foundation/Blender/{0}/config'.format(blenderversion)
            )
        )
    else:
        scriptspath = 'ERROR: {0} not supported,'.format(sys.platform)
    return scriptspath
