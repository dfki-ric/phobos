#!/usr/bin/python
# coding=utf-8

"""
..module:: phobos.defs
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

..moduleauthor:: Kai von Szadowski

Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File defs.py

Created on 7 Jan 2014
"""

import os
import glob
import re

import yaml
from phobos.phoboslog import log

# Phobos information
version = '0.7'
repository = 'https://github.com/dfki-ric/phobos'

# definitions of which elements are assigned to which default layers
layerTypes = {
    "link": 0,
    'inertial': 1,
    "visual": 2,
    "collision": 3,
    "sensor": 4,
    "decoration": 5,
    "light": 6,
    "approxsphere": 13,
    'interface': 10,
    'submodel': 10
}

# types of blender objects phobos differentiates
phobostypes = (('undefined',) * 3,
               ('link',) * 3,
               ('inertial',) * 3,
               ('visual',) * 3,
               ('collision',) * 3,
               ('sensor',) * 3,
               ('controller',) * 3,
               ('approxsphere',) * 3,
               ('light',) * 3,
               ('entity',) * 3,
               ('frame',) * 3,
               ('interface',) * 3,
               ('submodel',) * 3,
               ('annotation',) * 3
               )

jointtypes = (('revolute',) * 3,
              ('continuous',) * 3,
              ('prismatic',) * 3,
              ('fixed',) * 3,
              ('floating',) * 3,
              ('planar',) * 3)

geometrytypes = (('box',) * 3,
                 ('cylinder',) * 3,
                 ('sphere',) * 3,
                 ('capsule',) * 3,
                 ('mesh',) * 3)

linkobjignoretypes = {'link', 'joint', 'motor', 'submechanism', 'entity'}

type_properties = {"undefined": (),
                   "undefined_default": (),
                   "link": ('name',),
                   "link_default": ('new_node',),
                   "inertial": ('name',),
                   "inertial_default": ('new_node',),
                   "visual": ('name', 'visual/type'),
                   "visual_default": ('new_node', 'box'),
                   "collision": ('name', 'collision/type', 'bitmask'),
                   "collision_default": ('new_node', 'box', '65536'),
                   "approxsphere": (),
                   "approxsphere_default": (),
                   "sensor": ('name', 'sensor/type'),
                   "sensor_default": ("new_sensor", "RaySensor"),
                   "controller": ('name',),
                   "controller_default": ("controller",),
                   "light": ('name', 'light/directional', 'light/exponent'),
                   "light_default": ('new_light', 'true', '1.0'),
# TODO we should handle these someway different when actually using submodels
                   "interface": (),
                   "interface_default": (),
                   "submodel": (),
                   "submodel_default": ()
                   }

# definitions of model elements to be read in
definitions = {'motors': {},
               'sensors': {},
               'controllers': {},
               'algorithms': {},
               'materials': {},
               'model': {},
               'submechanisms': {},
               'submodeltypes': {}
               }


def updateDefs(defsFolderPath):
    """Updates the definitions with all yml files in the given folder

    :param defsFolderPath: path to the folder with yaml files for definitions
    :type defsFolderPath: str
    """
    dicts = __parseAllYAML(defsFolderPath)
    for diction in dicts:
        for category in diction:
            for key, value in diction[category].items():
                if category not in definitions:
                    definitions[category] = {}
                if key in definitions[category]:
                    log("Entry for "+category+'/'+key+" will be overwritten while parsing definitions.", "WARNING")
                definitions[category][key] = value
    # Extending model definition
    definitions['model']['sensors']['$forElem']['$selection__type'] = definitions['sensors']


def __evaluateString(s):
    """Evaluates a string by searching for mathematical expressions enclosed
    in '&' and evaluating the inner string as python code.

    :param s: The string to evaluate.
    :type s: str
    :return: str -- the evaluated string.
    """
    # TODO math is not needed anymore...
    # needed for evaluation of strings (see below)
    import math
    p = re.compile('&.*&')
    for ma in p.findall(s):
        try:
            s = s.replace(ma, str(eval(ma[1:-1])))
        except():
            log("The expression " + ma + " could not be evaluated. Ignoring file", "ERROR")
            return ""
    return s


def __parseAllYAML(path):
    """Reads all .yml files in the given path and loads them.

    Expressions are evaluated if they are enclosed by '&' in those files.

    :param path: path from which to parse all files
    :type path: str
    :return: dictionary containing all parsed YAML files
    :rtype: dict
    """
    dicts = []
    for file in glob.iglob(os.path.join(path, '**/*.yml'), recursive=True):
        print('  ' + os.path.basename(file))
        try:
            file = open(file, 'r')
            tmpstring = file.read()
            file.close()

            try:
                tmpyaml = yaml.load(__evaluateString(tmpstring))

                if not tmpyaml:
                    log(file + " does not contain any yaml information.", 'ERROR')
                    continue
                dicts.append(tmpyaml)
            except yaml.scanner.ScannerError:
                log(file + " could not be parsed!", 'ERROR')
        except FileNotFoundError:
            log(file + " was not found.", 'ERROR')
    return dicts


# Update definitions from files
print("Parsing definitions from: " + os.path.dirname(__file__) + "/definitions")
updateDefs(os.path.dirname(__file__) + "/definitions")
