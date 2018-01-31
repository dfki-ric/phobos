#!/usr/bin/env python3

# Copyright 2017, University of Bremen & DFKI GmbH Robotics Innovation Center
#
# This file is part of Phobos, a Blender Add-On to edit robot models.
#
# Phobos is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3
# of the License, or (at your option) any later version.
#
# Phobos is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Phobos.  If not, see <http://www.gnu.org/licenses/>.
#
# File install_phobos.py
#
# author: Simon Reichel
#
# This script will install phobos into your user bound python addon repository.
#
# If you used the old install script you can delete the previous phobos folder.
# You also have to delete the old install.conf to run this script
# correctly.

import os.path as path
import sys
import os
import shutil

configfile = 'install.conf'
addonpath = 'ERROR'


def copytree(src, dst, symlinks=False, ignore=None):
    os.makedirs(dst)
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)


def makeConfigFile(blenderversion=None, pythoncommand=None):
    operating_system = sys.platform
    if not blenderversion:
        blenderversion = input('Enter your Blender version (e.g. "2.78") ')
    global addonpath

    if operating_system == 'linux':
        addonpath = path.normpath(path.expanduser(
            '~/.config/blender/{0}/scripts/addons'.format(blenderversion)))
    elif operating_system == 'darwin':
        addonpath = path.normpath(path.expanduser(
            '~/Library/Application Support/Blender/{0}/scripts/addons'.format(blenderversion)))
    elif operating_system == 'win32':
        addonpath = path.normpath(path.expanduser(
            '~/AppData/Roaming/Blender Foundation/Blender/{0}/scripts/addons'.format(blenderversion)))
    else:
        addonpath = ('ERROR: System not supported yet:' +
                     ' "{0}". Please contact the developers.').format(operating_system)
    
    if not pythoncommand:
        pythoncommand = input('What is your Python 3 command? (e.g. python3) ')
    # use default on empty entry
    if not pythoncommand:
        pythoncommand = 'python3'
    with open(configfile, 'w') as conffile:
        conffile.writelines([
            'blenderversion={0}\n'.format(blenderversion),
            'pythoncom={0}\n'.format(pythoncommand),
            'addonpath={0}\n'.format(addonpath)])


def copyphobos(phobospath):
    try:
        # remove old installation first
        if os.path.exists(phobospath):
            shutil.rmtree(phobospath)

        copytree(path.join(os.getcwd(), 'phobos'), phobospath)
        print('Phobos installation found and updated.')
        print('Copied Phobos to ' + phobospath)
        return True
    except shutil.Error:
        import traceback
        print('Error in copying the phobos data.')
        traceback.print_exc()
        return False


def installPhobos():
    # find phobos installation
    if addonpath[:5] == 'ERROR':
        print('Installation aborted: missing addonpath.')
        return False
    global phobospath
    phobospath = path.join(addonpath, 'phobos')

    if path.isdir(phobospath):
        return copyphobos(phobospath)
    else:
        os.makedirs(phobospath)
        return copyphobos(phobospath)


if __name__ == '__main__':
    # digest parameters
    print(sys.argv)
    version = None
    pythoncommand = None
    if len(sys.argv) > 1:
        version = sys.argv[1]
        pythoncommand = sys.argv[2]
    
    # work always from installation folder
    os.chdir(os.path.abspath(os.path.dirname(__file__)))
    # check for existing configfile
    if not version and not pythoncommand:
        if path.isfile(configfile):
            print('Found installation configuration.')
            with open(configfile, 'r') as conffile:
                lines = conffile.readlines()
                addonpath = lines[2].split('=')[1].strip('\n')
        else:
            makeConfigFile()
    else:
        makeConfigFile(version, pythoncommand)

    # install Phobos
    if not installPhobos():
        print('Installation aborted.')
    else:
        print('Phobos installed.')

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

            yamlpath = yaml.__file__

            # write python dist packages path into config file
            with open('python_dist_packages.conf', 'w') as distconffile:
                distconffile.truncate()
                distpath = os.path.split(os.path.split(yamlpath)[0])[0]
                distconffile.write(path.normpath(distpath))

        shutil.copy2('python_dist_packages.conf', os.path.join(phobospath,
            'python_dist_packages.conf'))

