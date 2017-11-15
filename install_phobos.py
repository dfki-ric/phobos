#! /usr/bin/python3

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
    if os.path.exists(dst):
        shutil.rmtree(dst)

    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)


def makeConfigFile():
    operating_system = sys.platform
    blenderversion = input('Enter your Blender version (e.g. "2.78") ')
    global addonpath

    if operating_system == 'linux':
        addonpath = path.expanduser(('~/.config/blender/{0}/scripts/' +
                                     'addons').format(blenderversion))
    # CHECK works with darwin?
    elif operating_system == 'darwin':
        addonpath = path.expanduser(('~/Library/Application\ Support/' +
                                     'Blender/{0}/scripts/addons'.format(
                                         blenderversion)))
    # CHECK works with Windows?
    elif operating_system == 'win32':
        addonpath = path.expanduser(('~/AppData/Roaming/Blender\ Foundation' +
                                     '/{0}/scripts/addons'.format(
                                         blenderversion)))
    else:
        addonpath = ('ERROR: System not supported yet: "{0}". Please contact' +
                     ' the developers.').format(operating_system)

    pythoncommand = input('What is your Python 3 command? (e.g. python3) ')
    with open(configfile, 'w') as conffile:
        conffile.writelines([
            'blenderversion={0}\n'.format(blenderversion),
            'pythoncom={0}\n'.format(pythoncommand),
            'addonpath={0}\n'.format(addonpath)])


def copyphobos(phobospath):
    try:
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
        yn = input(('Phobos folder does not exist, create phobos folder ' +
                    'in {0}? (y/n) ').format(phobospath))
        if yn == 'y':
            os.makedirs(phobospath)
            return copyphobos(phobospath)
        else:
            print('No folder for Phobos created.')
            return False


if __name__ == '__main__':
    # check for existing configfile
    if path.isfile(configfile):
        print('Found installation configuration.')
        with open(configfile, 'r') as conffile:
            lines = conffile.readlines()
            addonpath = lines[2].split('=')[1].strip('\n')
    else:
        makeConfigFile()

    # install Phobos
    if not installPhobos():
        print('Installation aborted.')
    else:
        print('Phobos installed.')

        # look for existing yamlpath configuration
        if path.isfile('yamlpath.conf'):
            print('yamlpath.conf found! Configuration done.')
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

            yamlpath = yaml.__file__

            # write yamlpath into config file
            with open('yamlpath.conf', 'w') as yamlconffile:
                yamlconffile.truncate()
                yamlconffile.write(yamlpath.strip('__init__.py'))

        shutil.copy2('yamlpath.conf', os.path.join(phobospath, 'yamlpath.conf'))

