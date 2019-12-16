#!/usr/bin/python

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
This module should be run to install Phobos properly.

It moves the required files to the Blender config folder of your system and also installs the
ressources etc to your user config folder.

Make sure you run it with the appropriate python version (3.5).
"""

import os
import os.path as path
import glob
import sys
import shutil
import argparse
from distutils.dir_util import copy_tree
from importlib import util

# make installation originate from the path of this setup file
phoboshome = path.dirname(path.abspath(__file__))

# load the phobossystem as module from file
module_spec = util.spec_from_file_location(
    'phobossystem', path.join(phoboshome, 'phobos/phobossystem.py')
)
phobossystem = util.module_from_spec(module_spec)
module_spec.loader.exec_module(phobossystem)
addonpath = path.join(phobossystem.getScriptsPath(), 'addons', 'phobos')
blenderconfigpath = phobossystem.getBlenderConfigPath()


def updateFolderContents(src, dst):
    """Updates the directory tree at dst with everything from src.

    Args:
      src: source path
      dst: destination path

    Returns:

    """
    return copy_tree(src, dst, update=True, verbose=True, dry_run=False)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='''
        This is the setup script for Phobos.

        By default, it installs the Blender AddOn Phobos to the Blender configuration folder.
        At the same time, the default configurations are copied to the user Phobos configuration
        folder.''')
    parser.add_argument('-p', '--startup-preset', dest='startup_preset',
                        action='store_true', default=False,
                        help='''Copies the default Phobos Blender startup file to the Blender
                            'configuration folder (replacing the existing startup file).''')
    parser.add_argument('-t', '--install-to', dest='install_to', metavar='DIR', default=None,
                        help='install addon to the specified directory')
    parser.add_argument('-b', '--blender', metavar='DIR', dest='blender_path',
                        default=None, help='use the specified Blender folder (instead of asking)')
    parser.add_argument('-i', '--info', dest='print_info', action='store_true',
                        default=False, help='print debugging info (such as blender version etc.)')

    globals().update(vars(parser.parse_args()))

    # look for existing installation configuration
    if path.isfile(path.join(phoboshome, 'installation.conf')):
        print('installation.conf found! Configuration done.')
        with open(path.join(phoboshome, 'installation.conf'), 'r') as conffile:
            python_package_path = conffile.readline().split(' #')[0]
            if not blender_path:
                blender_path = conffile.readline().split(' #')[0]
            else:
                conffile.readline()
            python_executable = conffile.readline().split(' #')[0]
            blender_executable = conffile.readline().split(' #')[0]
            python_version = conffile.readline().split(' #')[0]
            blender_version = conffile.readline().split(' #')[0]
    # check for existing YAML installation
    else:
        if not blender_path:
            blender_path = path.expanduser(input('Where is Blender installed to? '))
        blender_version = glob.glob(path.join(blender_path, '[0-9].[0-9][0-9]'))
        if not blender_version or not path.isdir(blender_version[0]):
            print("Could not identify the blender subfolder.")
            print("Installation aborted!")
            sys.exit(1)
        blender_version = path.basename(blender_version[0])
        blender_executable = path.join(blender_path, 'blender')
        if not path.isfile(blender_executable):
            print("Could not find Blender executable.\nInstallation aborted.")
            sys.exit(1)

        bpython_files = glob.glob(path.join(
            blender_path, blender_version, 'python', 'bin', 'py*'))
        if not bpython_files:
            print("Could not find python executable in blender installation.")
            print("Installation aborted!")
            sys.exit(1)
        python_executable = bpython_files[0]
        python_version = path.basename(python_executable).strip('m').strip('python')
        python_package_path = path.normpath((path.join(
            blender_path, blender_version, 'python', 'lib', 'python' + python_version,
            'site-packages')))

        print('Installing PIP...')
        os.system(python_executable + ' -m ensurepip')
        os.system(python_executable + ' -m pip install --upgrade pip')
        print("... successful.\n")

        print('Installing YAML...')
        os.system(python_executable + ' -m pip install PyYaml')
        os.system(python_executable + ' -m pip install --upgrade PyYaml')
        print("... successful.\n")

        # write python dist packages path into config file
        with open(path.join(phoboshome, 'installation.conf'), 'w') as distconffile:
            distconffile.truncate()
            distconffile.write(python_package_path + ' # Python package installation path\n')
            distconffile.write(blender_path + ' # Blender installation path\n')
            distconffile.write(python_executable + ' # python executable\n')
            distconffile.write(blender_executable + ' # Blender executable\n')
            distconffile.write(python_version + ' # Python version\n')
            distconffile.write(blender_version + ' # Blender version\n')

    if print_info:
        print('''
Debug information for Phobos:
  - Blender version {} (installed in {})
  - Python version {} (located in {})
  - Python site packages are in {}:
    - {}
              '''.format(blender_version, blender_path, python_version,
                         python_executable, python_package_path,
                         '\n    - '.join(package for package in os.listdir(python_package_path))))
        sys.exit(0)

    shutil.copy2(path.join(phoboshome, 'installation.conf'),
                 path.join(addonpath, 'installation.conf'))

    if install_to:
        addonpath = install_to

    # install addon
    if path.exists(addonpath):
        shutil.rmtree(addonpath)  # always clean install folder
        print('Removed older installation.')

    copied_files = updateFolderContents(path.join(phoboshome, 'phobos'), addonpath)
    if not copied_files:
        print('Something went wrong with copying the addon files to your Blender installation.\n',
              'Aborting installation.')
        sys.exit(1)
    else:
        print('Copied addon files to ' + addonpath + '.')

    # install optional startup blend
    if startup_preset:
        shutil.copy(path.join(phoboshome, 'config', 'startup.blend'), blenderconfigpath)

    # install config files
    copied_files = updateFolderContents(
        path.join(phoboshome, 'config'), phobossystem.getConfigPath())
    if not copied_files:
        print('Something went wrong with copying config files.')
    os.remove(path.join(phobossystem.getConfigPath(), 'startup.blend'))

    # install templates
    templatespath = path.join(phobossystem.getScriptsPath(), 'templates_py')
    copied_files = updateFolderContents(path.join(phoboshome, 'templates_py'), templatespath)
    if not copied_files:
        print('Something went wrong with copying operator presets.\nInstallation aborted.')
        sys.exit(1)

    print('Installation successful!')

    # # install presets
    # presetspath = path.join(phobossystem.getScriptsPath(), 'presets', 'operator')
    # copied_files = updateFolderContents(path.join(phoboshome, 'presets'), presetspath)
    # if not copied_files:
    #    print('Something went wrong with copying operator presets.')

"""
Thoughts on importing external packages:
- Blender brings its own python
- The system python version cannot be assumed to be the same as the internal in blender
- thus linking to the systems site package version only works for the coincidence of equal versions

Possible other solutions:
  a) setup virtual environment somewhere in the system and add to blender python's path environment

Solutions in detail:

a) This script could set up a virtual environment with the correct python version and install all
   packages, then place a file containing the path of that environment to phobos, which would expand
   the path upon startup.  In fact, the install file itself could set up the environment, asking the
   user where to put it. Alternatively and more simply, the environment could be created where the
   file it situated. That saves us one temp file to store the path of the environment in.
"""
