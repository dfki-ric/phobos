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
from datetime import datetime
import mathutils
from phobos.phoboslog import log
import os
import shutil


def is_float(s):
    """Tests if an input variable (string) is a float number.

    Args:
      s: 

    Returns:

    """
    try:
        float(s)
        return True
    except (ValueError, TypeError):
        return False


def is_int(s):
    """Tests if an input variable (string) is an int number.

    Args:
      s: 

    Returns:

    """
    try:
        int(s)
        return True
    except ValueError:
        return False


def parse_number(s):
    """Takes an input variable (string) and determines whether it represents
    a float number, int or string

    Args:
      s: 

    Returns:

    """
    if is_int(s):
        return int(s)
    elif is_float(s):
        return float(s)
    else:
        return s


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively.

    Args:
      stringlist: 

    Returns:

    """
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively.

    Args:
      stringlist: 

    Returns:

    """
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def find_in_list(alist, prop, value):
    """Returns the index of the first object in a list which has a field
    named *prop* with value *value*. If no such object is found, returns -1.

    Args:
      alist: 
      prop: 
      value: 

    Returns:

    """
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            log("The object at index " + str(i) + " has no property " + str(prop), "ERROR")
    return n


def retrieve_from_list(alist, prop, value):
    """Returns the first object in a list which has a field named
    *prop* with value *value*. If no such object is found, returns 'None'.

    Args:
      alist: 
      prop: 
      value: 

    Returns:

    """
    n = -1
    for i in range(len(alist)):
        try:
            if alist[i][prop] == value:
                n = i
                break
        except KeyError:
            log("The object at index " + str(i) + " has no property " + str(prop), "ERROR")
    if n >= 0:
        return alist[n][prop]
    else:
        return "None"


def parse_text(s):
    """Parses a text by splitting up elements separated by whitespace and tries
    to determine whether it is a list of floats, ints or strings.

    Args:
      s: 

    Returns:

    """
    numstrings = s.split()
    if not numstrings:
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
    """Calculates the center of a bounding box

    Args:
      boundingbox: 

    Returns:

    """
    c = sum((mathutils.Vector(b) for b in boundingbox), mathutils.Vector())
    return c / 8


def sortListsInDict(data, reverse=False):
    """Recursively loops through a dictionary and sorts all lists.

    Args:
        data(dict): data dictionary

    Returns:
        dict -- sorted dictionary
    """
    if isinstance(data, list):
        if not data:
            return data

        return sorted(data, reverse=reverse)
    elif isinstance(data, dict):
        return {key: sortListsInDict(value, reverse) for key, value in data.items()}
    else:  # any other type, such as string
        return data


def roundFloatsInDict(data, decimals):
    """Recursively rounds all floats in the dictionary to the specified decimal digits.

    If a float value is smaller than 10e-decimals it is set to zero.

    Args:
        data(dict): data dictionary
        decimals(int): number of decimals floats should be rounded to

    Returns:
        dict -- dictionary with rounded floats
    """
    epsilon = 10**-decimals
    if is_float(data):
        if type(data) == str:
            log("Skipping rounding of " + data + " due to its type 'str'", "WARNING")
            return data
        return 0 if abs(data) < epsilon else round(data, decimals)
    elif type(data) is list:
        return [roundFloatsInDict(a, decimals) for a in data]
    elif type(data) is dict:
        return {key: roundFloatsInDict(value, decimals) for key, value in data.items()}
    else:  # any other type, such as string
        return data


def calculateSum(objects, numeric_prop):
    """Returns sum of *numeric_prop* in *objects*.

    Args:
        objects (list(bpy.types.Object)): objects to sum up the property for
        numeric_prop (str): name of the custom property to sum

    Returns:
        float/int -- sum of the values in the property of the objects
    """
    numsum = 0
    for obj in objects:
        try:
            numsum += obj[numeric_prop]
        except KeyError:
            log("{0} object {1} does not contain '{2}'".format(obj.phobostype, obj.name,
                                                               numeric_prop), "WARNING")
        except TypeError:
            import phobos.utils.naming as nUtils
            log("Could not add this type to the sum: " + str(type(obj[numeric_prop])) + " @" +
                nUtils.getObjectName(obj), 'WARNING')
    return numsum


def datetimeFromIso(iso):
    """Accepts a date-time string in iso format and returns a datetime object.

    Args:
      iso: 

    Returns:

    """
    try:
        dtime = datetime(*[int(a) for a in re.split(":|-|T| |\.", iso)])
        return dtime
    except ValueError as e:
        log("Could not convert iso string: "+str(e), "ERROR")
        return datetime.now()


def distance(objects):
    """Returns the distance between two blender objects.

    Args:
      objects(list -- with exactly two elements): The two objects to calculate the distance for.

    Returns:

    """
    v = objects[0].matrix_world.to_translation() - objects[1].matrix_world.to_translation()
    return v.length, v


def outerProduct(v, u):
    """Returns a mathutils.Matrix representing the outer product of vectors v and u.

    Args:
      v: 
      u: 

    Returns:

    """
    lines = []
    for vi in v:
        lines.append([vi * ui for ui in u])
    return mathutils.Matrix(lines)


def copyTree(src, dst, symlinks=False, ignore=None):
    """Copies the folder tree from src to dst.

    Args:
      src: 
      dst: 
      symlinks:  (Default value = False)
      ignore:  (Default value = None)

    Returns:

    """
    if not os.path.exists(dst):
        os.makedirs(dst)

    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)

