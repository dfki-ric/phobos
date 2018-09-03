# TODO SHEBANG

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

import sys
import os.path as path

# compatible blender version
blenderversion = '2.79'


def getScriptsPath():
    """Returns the path for user-specific blender scripts for all major platforms

    Returns(str): scripts path
    """
    if sys.platform == 'linux':
        scriptspath = path.normpath(path.expanduser(
            '~/.config/blender/{0}/scripts'.format(blenderversion)))
    elif sys.platform == 'darwin':
        scriptspath = path.normpath(path.expanduser(
            '~/Library/Application Support/Blender/{0}/scripts'.format(blenderversion)))
    elif sys.platform == 'win32':
        scriptspath = path.normpath(path.expanduser(
            '~/AppData/Roaming/Blender Foundation/Blender/{0}/scripts'.format(blenderversion)))
    else:
        scriptspath = 'ERROR: {0} not supported,'.format(sys.platform)
    return scriptspath


def getConfigPath():
    """Returns the path for configuration data for all major platforms

    Returns(str): config path
    """
    if sys.platform == 'linux':
        configpath = path.normpath(path.expanduser('~/.config/phobos'))
    elif sys.platform == 'darwin':
        configpath = path.normpath(path.expanduser('~/Library/Application Support/phobos'))
    elif sys.platform == 'win32':
        configpath = path.normpath(path.expanduser('~/AppData/Roaming/phobos'))
    else:
        configpath = 'ERROR: {0} not supported,'.format(sys.platform)
    return configpath


def getBlenderConfigPath():
    """Returns the configuration path for user-specific blender data.

    Returns:
        str -- scripts path
    """
    if sys.platform == 'linux':
        scriptspath = path.normpath(path.expanduser(
            '~/.config/blender/{0}/config'.format(blenderversion)))
    elif sys.platform == 'darwin':
        scriptspath = path.normpath(path.expanduser(
            '~/Library/Application Support/Blender/{0}/config'.format(blenderversion)))
    elif sys.platform == 'win32':
        scriptspath = path.normpath(path.expanduser(
            '~/AppData/Roaming/Blender Foundation/Blender/{0}/config'.format(blenderversion)))
    else:
        scriptspath = 'ERROR: {0} not supported,'.format(sys.platform)
    return scriptspath
