#!/usr/bin/env python3.5

import os
import os.path as path
import sys
import shutil
from distutils.dir_util import copy_tree
from distutils.errors import DistutilsFileError

blenderversion = '2.79'

def updateFolderContents(src, dst):
    return copy_tree(src, dst, update=True, verbose=True, dry_run=False)


def getScriptsPath():
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
        scriptspath = ('ERROR: System not supported yet:' +
                     ' "{0}". Please contact the developers.').format(sys.platform)
    return scriptspath


def getResourcesPath():
    if sys.platform == 'linux':
        resourcespath = path.normpath(path.expanduser('~/.config/phobos/resources'))
    elif sys.platform == 'darwin':
        resourcespath = path.normpath(path.expanduser('~/Library/Application Support/phobos/resources'))
    elif sys.platform == 'win32':
        resourcespath = path.normpath(path.expanduser('~/AppData/Roaming/phobos/resources'))
    else:
        resourcespath = ('ERROR: System not supported yet:' +
                     ' "{0}". Please contact the developers.').format(sys.platform)
    return resourcespath


addonpath = path.join(getScriptsPath(), 'addons', 'phobos')


if __name__ == '__main__':
    # check whether the right python version is used
    pyver = sys.version_info
    if pyver.major != 3 or pyver.minor != 5:
        print('You started this script with the wrong python version: ')
        print('Current: ' + str(pyver.major) + '.' + str(pyver.minor))
        print('Blender python uses: 3.5')
        print('Installation aborted.')
        sys.exit(0)

    phoboshome = os.path.dirname(os.path.abspath(__file__))

    # install addon
    if os.path.exists(addonpath):
        shutil.rmtree(addonpath)  # always clean install folder
    copied_files = updateFolderContents(
        os.path.join(phoboshome, 'phobos'),
        addonpath)
    if not len(copied_files) > 0:
        print('Something went wrong with copying the addon files to your Blender installation.\n',
              'Aborting installation.')
        sys.exit(0)

    ## install resources
    #copied_files = updateFolderContents(os.path.join(phoboshome, 'resources'), getResourcesPath())
    #if not len(copied_files) > 0:
    #    print('Something went wrong with copying resource files.')

    # install templates
    templatespath = path.join(getScriptsPath(), 'templates_py')
    copied_files = updateFolderContents(
        os.path.join(phoboshome, 'templates_py'),
        templatespath)
    if not len(copied_files) > 0:
        print('Something went wrong with copying operator presets.')

    ## install presets
    #presetspath = path.join(getScriptsPath(), 'presets', 'operator')
    #copied_files = updateFolderContents(os.path.join(phoboshome, 'presets'), presetspath)
    #if not len(copied_files) > 0:
    #    print('Something went wrong with copying operator presets.')

    # look for existing yamlpath configuration
    if path.isfile('python_dist_packages.conf'):
        print('python_dist_packages.conf found! Configuration done.')
    # check for existing YAML installation
    else:
        try:
            import yaml
        except ImportError:
            print('YAML installation not found. Please install it first:' +
                  '\n\n' +
                  '  pip3 install PyYaml\n\n')
            print('Please make sure you followed the instructions on ' +
                  'https://github.com/rock-simulation/phobos/wiki/Installation')
            print('YAML configuration aborted. Installation incomplete.')
            sys.exit(0)
        # OPT here we could add additional requirement checks

        import site

        # write python dist packages path into config file
        with open('python_dist_packages.conf', 'w') as distconffile:
            distconffile.truncate()
            distpath = site.getsitepackages()[0]
            distconffile.write(path.normpath(distpath))

    shutil.copy2('python_dist_packages.conf', os.path.join(addonpath,
        'python_dist_packages.conf'))


"""
Thoughts on importing external packages:
- Blender brings its own python
- The system python version cannot be assumed to be the same as the internal in blender
- thus linking to the systems site package version only works for the coincidence of equal versions

Possible solutions:
  a) install packages of same python version as blender's internal to internal site packages
  b) setup virtual environment somewhere in the system and add to blender python's path environment
  
Solutions in detail:

a) Install into blender python

   This is generally possible by executing:
   python3.5 -m pip install --target /path/to/blender packagename
   example:
   python3.5 -m pip install --target =/home/dfki.uni-bremen.de/kavonszadkowski/tools/blender-2.79/2.79/python/lib/python3.5/site-packages packagename

   The /path/to/blender can be retrieved from within blender by executing:
        import site
        sitepackagepath = site.getsitepackages()[0]
        
    This could be retrieved to this script by calling blender as follows:
    blender -b -P script.py
    
    with script.py writing out the sitepackagepath and closing blender.
    For this, however, the appropriate binary for blender is required, which would already suffice to derive the path.
    
    Another alternative would thus be to try importing external packages upon phobos startup, and run a shell command
    to install missing packages of the correct version to blender's then-known-location by using --target.
    
b) This script could set up a virtual environment with the correct python version and install all packages, then place
   a file containing the path of that environment to phobos, which would expand the path upon startup.
   In fact, the install file itself could set up the environment, asking the user where to put it. Alternatively
   and more simply, the environment could be created where the file it situated. That saves us one
   temp file to store the path of the environment in.
"""

