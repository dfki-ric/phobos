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
import os
import subprocess

def getPythonExecutable():
    if sys.platform == 'linux':
        python_path = subprocess.check_output('which python3', shell=True).decode().rstrip()
        return python_path
    else:
        return None

def getPythonVersion():
    if sys.platform == 'linux':
        python_path = getPythonExecutable()
        python_version_raw = subprocess.check_output(f'{python_path} --version', shell=True).decode().rstrip()
        # python version raw is 'Python X.Y.Z'
        python_version_splitted = python_version_raw.split(" ")
        # The long version might be like X.Y.Z
        python_version_long = python_version_splitted[1]
        python_major, python_minor, python_sub = python_version_long.split(".")
        python_major = int(python_major)
        python_minor = int(python_minor)
        python_sub = int(python_sub)
        if python_major < 3:
            return None
        if python_major == 3 and python_minor < 5:
            return None
        return (python_major, python_minor, python_sub)
    else:
        return None

def getBlenderExecutable():
    if sys.platform == 'linux':
        blender_path = subprocess.check_output('which blender', shell=True).decode().rstrip()
        return blender_path
    else:
        return None

def getBlenderVersion():
    if sys.platform == 'linux':
        blender_path = getBlenderExecutable()
        blender_version_raw = subprocess.check_output(f'{blender_path} --version', shell=True).decode().rstrip()
        # The string is something like "Blender X.YZ (...)"
        blender_version_splitted = blender_version_raw.split(" ")
        if len(blender_version_splitted) < 2:
            return None
        blender_version_long = blender_version_splitted[1]
        blender_major, blender_minor = blender_version_long.split(".")
        blender_major = int(blender_major)
        blender_minor = int(blender_minor)
        if blender_major < 2:
            return None
        if blender_major == 2 and blender_minor < 79:
            return None
        return (blender_major, blender_minor)
    else:
        return None

def getScriptsPath(blenderversion):
    """Returns the path for user-specific blender scripts for all major platforms
    
    Returns(str): scripts path

    Args:
      : blenderversion -- the blender version to use

    Returns:

    """
    if sys.platform == 'linux':
        scriptspath = os.path.normpath(
            os.path.expanduser('~/.config/blender/{0}/scripts'.format(blenderversion))
        )
    elif sys.platform == 'darwin':
        scriptspath = os.path.normpath(
            os.path.expanduser(
                '~/Library/Application Support/Blender/{0}/scripts'.format(blenderversion)
            )
        )
    elif sys.platform == 'win32':
        scriptspath = os.path.normpath(
            os.path.expanduser(
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
    if sys.platform == 'linux':
        configpath = os.path.normpath(os.path.expanduser('~/.config/phobos'))
    elif sys.platform == 'darwin':
        configpath = os.path.normpath(os.path.expanduser('~/Library/Application Support/phobos'))
    elif sys.platform == 'win32':
        configpath = os.path.normpath(os.path.expanduser('~/AppData/Roaming/phobos'))
    else:
        configpath = 'ERROR: {0} not supported,'.format(sys.platform)
    return configpath


def getBlenderConfigPath(blenderversion):
    """Returns the configuration path for user-specific blender data.

    Args:
      : blenderversion -- the blender version to use

    Returns:
      : str -- scripts path

    """
    if sys.platform == 'linux':
        scriptspath = os.path.normpath(
            os.path.expanduser('~/.config/blender/{0}/config'.format(blenderversion))
        )
    elif sys.platform == 'darwin':
        scriptspath = os.path.normpath(
            os.path.expanduser(
                '~/Library/Application Support/Blender/{0}/config'.format(blenderversion)
            )
        )
    elif sys.platform == 'win32':
        scriptspath = os.path.normpath(
            os.path.expanduser(
                '~/AppData/Roaming/Blender Foundation/Blender/{0}/config'.format(blenderversion)
            )
        )
    else:
        scriptspath = 'ERROR: {0} not supported,'.format(sys.platform)
    return scriptspath
