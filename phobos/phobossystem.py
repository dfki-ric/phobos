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
