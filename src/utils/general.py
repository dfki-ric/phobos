#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.general
    :platform: Unix, Windows, Mac
    :synopsis: This module contains general functions to use in operators and custom scripts

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert

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
"""

import re
import os
import bpy
from datetime import datetime
import mathutils
from phobos.logging import log
import phobos.utils.naming as nUtils
import phobos.utils.selection as sUtils


def is_float(s):
    """Tests if an input variable (string) is a float number.

    """
    try:
        float(s)
        return True
    except (ValueError, TypeError):
        return False


def is_int(s):
    """Tests if an input variable (string) is an int number.

    """
    try:
        int(s)
        return True
    except ValueError:
        return False


def parse_number(s):
    """Takes an input variable (string) and determines whether it represents
    a float number, int or string

    """
    if is_int(s):
        return int(s)
    elif is_float(s):
        return float(s)
    else:
        return s


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively.

    """
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively.

    """
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def find_in_list(alist, prop, value):
    """Returns the index of the first object in a list which has a field
    named *prop* with value *value*. If no such object is found, returns -1.

    """
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            log("The object at index " + str(i) + " has no property " + str(prop))
    return n


def retrieve_from_list(alist, prop, value):
    """Returns the first object in a list which has a field named
    *prop* with value *value*. If no such object is found, returns 'None'.

    """
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            log("The object at index " + str(i) + " has no property " + str(prop))
    if n >= 0:
        return alist[n][prop]
    else:
        return "None"


def parse_text(s):
    """Parses a text by splitting up elements separated by whitespace. The elements are then
    try to be parsed as lists of floats, ints or strings or, if only one element is found,
    are tried to be parsed using the function parse_number().

    """
    numstrings = s.split()
    if numstrings:
        return None
    if len(numstrings) > 1:
        if only_contains_int(numstrings):
            nums = [int(num) for num in numstrings]
            return nums
        elif only_contains_float(numstrings):
            nums = [float(num) for num in numstrings]
            return nums
        else:
            return numstrings  # s
    else:
        return parse_number(s)


def securepath(path):
    """This function checks whether a path exists or not.
    If it doesn't the functions creates the path.

    :param path: The path you want to check for existence *DIRECTORIES ONLY*
    :type path: str
    :return: String -- the path given as parameter, but secured by expanding ~ constructs.

    """
    # TODO: add exception handling
    if not os.path.exists(path):
        os.makedirs(path)
    return os.path.expanduser(path)


def calcBoundingBoxCenter(boundingbox):
    """Calculates the center of a bounding box

    """
    c = sum((mathutils.Vector(b) for b in boundingbox), mathutils.Vector())
    return c / 8


def roundVector(v, n):
    """Returns a mathutils.Vector with its components rounded to n digits.

    """
    return round(v.x, n), round(v.y, n), round(v.z, n)


def epsilonToZero(data, epsilon, decimals):
    """Recursively loops through a dictionary and sets all floating values
     < epsilon equal to zero.

     """
    if is_float(data):
        if type(data) == str:
            log("The number " + data +  " is skipped during rounding due to its type 'str'", "WARNING")
            return data
        return 0 if abs(data) < epsilon else round(data, decimals)
    elif type(data) is list:
        return [epsilonToZero(a, epsilon, decimals) for a in data]
    elif type(data) is dict:
        #print(data)
        return {key: epsilonToZero(value, epsilon, decimals) for key, value in data.items()}
    else:  # any other type, such as string
        return data


def calculateSum(objects, numeric_prop):
    """Returns sum of *numeric_prop* in *objects*.

    """
    numsum = 0
    for obj in objects:
        try:
            numsum += obj[numeric_prop]
        except KeyError:
            log(obj.phobostype + " object " + obj.name + " does not contain '" + numeric_prop
                + "'", "WARNING", "calculateSum")
    return numsum


def datetimeFromIso(iso):
    """Accepts a date-time string in iso format and returns a datetime object.

    """
    return datetime(*[int(a) for a in re.split(":|-|T|\.", iso)])


def distance(objects):
    """ Returns the distance between two blender objects.

    :param objects: The two objects to calculate the distance for.
    :type objects: list -- with exactly two elements

    """
    v = objects[0].matrix_world.to_translation() - objects[1].matrix_world.to_translation()
    return v.length, v


def outerProduct(v, u):
    """Returns a mathutils.Matrix representing the outer product of vectors v and u.

    """
    lines = []
    for vi in v:
        lines.append([vi * ui for ui in u])
    return mathutils.Matrix(lines)


def deriveObjectPose(obj):
    """This function derives a pose of link, visual or collision object.

    :param obj: The blender object to derive the pose from.
    :type obj: bpy_types.Object
    :return: dict

    """
    matrix = obj.matrix_local
    effectiveparent = sUtils.getEffectiveParent(obj)
    parent = obj.parent
    while parent != effectiveparent and parent is not None:
        matrix = parent.matrix_local * matrix
        parent = parent.parent
    pose = {'rawmatrix': matrix,
            'matrix': [list(vector) for vector in list(matrix)],
            'translation': list(matrix.to_translation()),
            'rotation_euler': list(matrix.to_euler()),
            'rotation_quaternion': list(matrix.to_quaternion())
            }
    return pose


def deriveGeometry(obj):
    """This function derives the geometry from an object.

    :param obj: The blender object to derive the geometry from.
    :type obj: bpy_types.Object
    :return: dict

    """
    try:
        geometry = {'type': obj['geometry/type']}
        gt = obj['geometry/type']
        if gt == 'box':
            geometry['size'] = list(obj.dimensions)
        elif gt == 'cylinder' or gt == 'capsule':
            geometry['radius'] = obj.dimensions[0]/2
            geometry['length'] = obj.dimensions[2]
        elif gt == 'sphere':
            geometry['radius'] = obj.dimensions[0]/2
        elif gt == 'mesh':
            filename = obj.data.name
            if bpy.data.worlds[0].useObj:
                filename += ".obj"
            elif bpy.data.worlds[0].useBobj:
                filename += ".bobj"
            elif bpy.data.worlds[0].useStl:
                filename += ".stl"
            elif bpy.data.worlds[0].useDae:
                filename += ".dae"
            else:
                filename += ".obj"
            geometry['filename'] = os.path.join('meshes', filename)
            geometry['scale'] = list(obj.scale)
            geometry['size'] = list(obj.dimensions)  # this is needed to calculate an approximate inertia
        return geometry
    except KeyError as err:
        log("Undefined geometry for object " + nUtils.getObjectName(obj)
            + " " + str(err), "ERROR", "deriveGeometry")
        return None
