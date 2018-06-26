#!/usr/bin/env python3.5
"""
This module should be run to install Phobos properly.

It moves the required files to the Blender config folder of your system and also installs the
ressources etc to your user config folder.

Make sure you run it with the appropriate python version (3.5).
"""

import os
import os.path as path
import sys
import shutil
from distutils.dir_util import copy_tree
import importlib.util

# make installation originate from the path of this setup file
phoboshome = os.path.dirname(os.path.abspath(__file__))

# load the phobossystem as module from file
module_spec = importlib.util.spec_from_file_location(
    'phobossystem', path.join(phoboshome, 'phobos/phobossystem.py'))
phobossystem = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(phobossystem)
addonpath = path.join(phobossystem.getScriptsPath(), 'addons', 'phobos')


def updateFolderContents(src, dst):
    """ Updates the directory tree at dst with everything from src.

    :param src: source path
    :param dst: destination path
    """
    return copy_tree(src, dst, update=True, verbose=True, dry_run=False)


if __name__ == '__main__':
    # check whether the right python version is used
    pyver = sys.version_info
    if pyver.major != 3 or pyver.minor != 5:
        print('You started this script with the wrong python version: ')
        print('Current: ' + str(pyver.major) + '.' + str(pyver.minor))
        print('Blender python uses: 3.5')
        print('Installation aborted.')
        sys.exit(0)

    # install addon
    if os.path.exists(addonpath):
        shutil.rmtree(addonpath)  # always clean install folder
    copied_files = updateFolderContents(
        os.path.join(phoboshome, 'phobos'),
        addonpath)
    if not copied_files:
        print('Something went wrong with copying the addon files to your Blender installation.\n',
              'Aborting installation.')
        sys.exit(0)

    # install config files
    copied_files = updateFolderContents(os.path.join(phoboshome, 'config'),
                                        phobossystem.getConfigPath())
    if not copied_files:
        print('Something went wrong with copying config files.')

    # install templates
    templatespath = path.join(phobossystem.getScriptsPath(), 'templates_py')
    copied_files = updateFolderContents(path.join(phoboshome, 'templates_py'), templatespath)
    if not copied_files:
        print('Something went wrong with copying operator presets.')

    # # install presets
    # presetspath = path.join(phobossystem.getScriptsPath(), 'presets', 'operator')
    # copied_files = updateFolderContents(path.join(phoboshome, 'presets'), presetspath)
    # if not copied_files:
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
                  'https://github.com/dfki-ric/phobos/wiki/Installation')
            print('YAML configuration aborted. Installation incomplete.')
            sys.exit(0)
        # OPT here we could add additional requirement checks

        import site

        # write python dist packages path into config file
        with open('python_dist_packages.conf', 'w') as distconffile:
            distconffile.truncate()
            distpath = site.getsitepackages()[0]
            distconffile.write(path.normpath(distpath))

    shutil.copy2('python_dist_packages.conf',
                 os.path.join(addonpath, 'python_dist_packages.conf'))


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
    For this, however, the appropriate binary for blender is required, which would already suffice
    to derive the path.

    Another alternative would thus be to try importing external packages upon phobos startup, and
    run a shell command to install missing packages of the correct version to blender's
    then-known-location by using
    --target.

b) This script could set up a virtual environment with the correct python version and install all
   packages, then place a file containing the path of that environment to phobos, which would expand
   the path upon startup.  In fact, the install file itself could set up the environment, asking the
   user where to put it. Alternatively and more simply, the environment could be created where the
   file it situated. That saves us one temp file to store the path of the environment in.
"""
