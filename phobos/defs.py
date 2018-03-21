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
import yaml
from re import compile
from phobos.phoboslog import log

# phobos version number
version = '0.8'

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
    'assembly': 10
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
               ('assembly',) * 3
               )

# DOCU add some comments for these...
subtypes = ("visual", "joint", "motor", "collision", "sensor", "link", "inertial", "controller", "light", "approxsphere")

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
                   "interface": (),
                   "interface_default": (),
                   "assembly": (),
                   "assembly_default": ()
                   }

# definitions of model elements to be read in
definitions = {'motors': {},
               'sensors': {},
               'controllers': {},
               'algorithms': {},
               'materials': {},
               'model': {},
               'submechanisms': {}
               }


def updateDefs(defsFolderPath):
    """Updates the definitions with all yml files in the given folder

    :param defsFolderPath: The path to the folder with the definitions yaml files.
    :type defsFolderPath: str
    """
    dicts = __parseAllYAML(defsFolderPath)
    for dict in dicts:
        for category in dict:
            for key, value in dict[category].items():
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
    p = compile('&.*&')
    for ma in p.findall(s):
        try:
            s = s.replace(ma, str(eval(ma[1:-1])))
        except():
            log("The expression " + ma + " could not be evaluated. Ignoring file", "ERROR")
            return ""
    return s


def __parseAllYAML(path):
    """Reads all .yml files in the given path and loads them.
    It also evaluates the expressions enclosed by '&' in those files.

    :param path: The path from which to parse all files.
    :type path: str
    :return: dict -- The dictionary with all parsed YAML files.
    """
    dicts = []
    for root, dirs, files in os.walk(path):
        for file in files:
            print('  '+file)
            if file.endswith(".yml"):
                try:
                    f = open(os.path.join(path, file), 'r')
                    tmpstring = f.read()
                    f.close()
                    try:
                        tmpyaml = yaml.load(__evaluateString(tmpstring))
                        dicts.append(tmpyaml)
                    except yaml.scanner.ScannerError:
                        log("Error while parsing YAML file", "ERROR")
                # TODO filenotfounderror is not imported or so...
                except FileNotFoundError:
                    log("The file "+file+" was not found.", "ERROR")
    return dicts


# Update definitions from files
print("Parsing definitions from: " + os.path.dirname(__file__) + "/definitions")
updateDefs(os.path.dirname(__file__) + "/definitions")
