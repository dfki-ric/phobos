#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains different definitions for Phobos. Additional defintions are parsed from YAML files and
added to this module at runtime.
"""

import glob
import json
import os
import re
import math # Required to parse yaml files

from . import phobossystem

# definitions of which elements are assigned to which default layers
layerTypes = {
    "link": 0,
    'inertial': 1,
    "visual": 2,
    "collision": 3,
    "sensor": 4,
    "decoration": 5,
    "light": 6,
    "controller": 8,
    "approxsphere": 13,
    'interface': 10,
    'submodel': 10,
    'annotation': 14,
    'submechanism': 15
}

# types of blender objects phobos differentiates
phobostypes = (
    ('undefined',) * 3,
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
    ('annotation',) * 3,
    ('submechanism',) * 3,
)

jointtypes = (
    ('revolute',) * 3,
    ('continuous',) * 3,
    ('prismatic',) * 3,
    ('fixed',) * 3,
    ('floating',) * 3,
    ('planar',) * 3,
)

geometrytypes = (('box',) * 3, ('cylinder',) * 3, ('sphere',) * 3, ('mesh',) * 3)

linkobjignoretypes = {'link', 'joint', 'submechanism', 'entity', 'model'}

type_properties = {
    "undefined": (),
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
    "submodel_default": (),
    # TODO add submechanism?
}

# definitions of model elements to be read in
definitions = {
    'sensors': {},
    'controllers': {},
    'algorithms': {},
    'materials': {},
    'model': {},
    'submechanisms': {},
    'submodeltypes': {},
}

def_settings = {key: {} for key in definitions.keys()}

def_subcategories = {key: set([]) for key in definitions.keys()}


def updateDefs(defsFolderPath):
    """Updates the definitions with all yml files in the given folder.

    Args:
      defsFolderPath(str): path to the folder with yaml files for definitions

    Returns:

    """
    assert os.path.isdir(defsFolderPath)
    dicts = __parseAllYAML(defsFolderPath)
    for diction in dicts:
        for category in diction:
            for key, value in diction[category].items():
                if category not in definitions:
                    print("Creating new definition type: " + category)
                    definitions[category] = {}
                    def_settings[category] = {}
                    def_subcategories[category] = set([])

                # TODO we need to insert user data, instead overwriting existing
                if key in definitions[category]:
                    print("Entry for " + category + '/' + key)

                # parse def_settings to other dictionary
                if isinstance(value, dict) and 'general' in value and value['general']:
                    def_settings[category][key] = value['general']
                    del value['general']
                else:
                    # add undefined stuff to other category
                    def_settings[category][key] = {'categories': ['other']}
                definitions[category][key] = value

    # update category sets
    for definition in def_settings.keys():
        for entry in def_settings[definition]:
            if 'categories' in def_settings[definition][entry]:
                categs = set(def_settings[definition][entry]['categories'])
                def_subcategories[definition] = def_subcategories[definition].union(categs)


def __evaluateString(s):
    """Evaluates a string by searching for mathematical expressions enclosed
    in '&' and evaluating the inner string as python code.

    :param s: The string to evaluate.
    :type s: str
    :return: str -- the evaluated string.
    """
    # TODO math is not needed anymore...
    # needed for evaluation of strings (see below)

    p = re.compile('&.*&')
    for ma in p.findall(s):
        try:
            s = s.replace(ma, str(eval(ma[1:-1])))
        except:
            print("The expression " + ma + " could not be evaluated. Ignoring file")
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
            with open(os.path.join(path, file), 'r') as f:
                tmpstring = f.read()

            try:
                tmpyaml = json.loads(__evaluateString(tmpstring))

                if not tmpyaml:
                    print(file + " does not contain any yaml information.")
                    continue
                dicts.append(tmpyaml)
            except Exception as e:
                print(os.path.relpath(file, path) + " could not be parsed:\n" + str(e))
        except FileNotFoundError:
            print(os.path.relpath(file, path=path) + " was not found.")
    return dicts


# Update definitions from files
definitionpath = os.path.join(phobossystem.getConfigPath(), 'definitions')
print("Parsing definitions from:", definitionpath)
updateDefs(definitionpath)
