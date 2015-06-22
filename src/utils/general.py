#!/usr/bin/python

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

from phobos.logging import log
import phobos.utils.selection as selectionUtils
import phobos.utils.naming as namingUtils
import datetime
import re
import mathutils


def is_float(s):
    """Tests if an input variable (string) is a float number."""
    try:
        float(s)
        return True
    except (ValueError, TypeError):
        return False


def is_int(s):
    """Tests if an input variable (string) is an int number."""
    try:
        int(s)
        return True
    except ValueError:
        return False


def parse_number(s):
    """Takes an input variable (string) and determines whether it represents
    a float number, int or string"""
    if is_int(s):
        return int(s)
    elif is_float(s):
        return float(s)
    else:
        return s


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively."""
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively."""
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def find_in_list(alist, prop, value):
    """Returns the index of the first object in a list which has a field
    named *prop* with value *value*. If no such object is found, returns -1."""
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
    *prop* with value *value*. If no such object is found, returns "None"."""
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
    are tried to be parsed using the function parse_number()."""
    numstrings = s.split()
    if numstrings == []:
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


def calcBoundingBoxCenter(boundingbox):
    """Calculates the center of a bounding box"""
    c = sum((mathutils.Vector(b) for b in boundingbox), mathutils.Vector())
    return c / 8


def roundVector(v, n):
    """Returns a mathutils.Vector with its components rounded to n digits."""
    return round(v.x, n), round(v.y, n), round(v.z, n)


def epsilonToZero(data, epsilon, decimals):
    """Recursively loops through a dictionary and sets all floating values
     < epsilon equal to zero."""
    if is_float(data):
        return 0 if abs(data) < epsilon else round(data, decimals)
    elif type(data) is list:
        return [epsilonToZero(a, epsilon, decimals) for a in data]
    elif type(data) is dict:
        print(data)
        return {key: epsilonToZero(value, epsilon, decimals) for key, value in data.items()}
    else:  # any other type, such as string
        return data


def calculateSum(objects, numeric_prop):
    """Returns sum of *numeric_prop* in *objects*."""
    numsum = 0
    for obj in objects:
        try:
            numsum += obj[numeric_prop]
        except KeyError:
            log("The object '" + namingUtils.getObjectName(obj) + "' has not property '" + numeric_prop + "'")
    return numsum


def datetimeFromIso(iso):
    """Accepts a date-time string in iso format and returns a datetime object."""
    return datetime(*[int(a) for a in re.split(":|-|T|\.", iso)])


def distance(objects):
    v = objects[0].matrix_world.to_translation() - objects[1].matrix_world.to_translation()
    return v.length, v


def outerProduct(v, u):
    """Returns a mathutils.Matrix representing the outer product of vectors v and u."""
    lines = []
    for vi in v:
        lines.append([vi * ui for ui in u])
    return mathutils.Matrix(lines)
