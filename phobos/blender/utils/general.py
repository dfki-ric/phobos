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
Contains the basic utility functions to round numbers, convert types etc.
"""

import os
import re
import shutil
from datetime import datetime

import mathutils

from ..phoboslog import log
from ..utils import naming as nUtils


def is_float(text):
    """Tests if the specified string represents a float number.

    Args:
      text(str): text to check

    Returns:
      : bool -- True if the text can be parsed to a float, False if not.

    """
    try:
        float(text)
        return True
    except (ValueError, TypeError):
        return False


def is_int(text):
    """Tests if the specified string represents an integer number.

    Args:
      text(str): text to check

    Returns:
      : bool -- True if the text can be parsed to an int, False if not.

    """
    try:
        int(text)
        return True
    except ValueError:
        return False


def parse_number(text):
    """Returns the specified string parsed to an int or float.
    
    If no number can be parsed, the original string is returned.
    
    To determine whether the number is an int or float, the functions `is_int` and `is_float` are
    used.

    Args:
      text(string): text to parse to a number

    Returns:
      : int/float/str -- depending on successful parsing, a number or a string is returned

    """
    if is_int(text):
        return int(text)
    elif is_float(text):
        return float(text)
    return text


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively.
    
    To determine whether the number is an int, the function `is_int` is used.

    Args:
      stringlist(list(str): list to check

    Returns:
      : bool -- True if every string in the list can be represented as int, False if not

    """
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively.
    
    To determine whether the number is a float, the function `is_float` is used.

    Args:
      stringlist(list(str): list to check

    Returns:
      : bool -- True if every string in the list can be represented as float, False if not

    """
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def parse_text(text):
    """Parses a text by splitting up elements separated by whitespace.
    
    The elements are then parsed to int/float-only lists.

    Args:
      text(str): text with elements seperated by whitespace

    Returns:
      : list(str/float/int) -- list with elements parsed to the same type

    """
    numstrings = text.split()
    if not numstrings:
        return None

    if len(numstrings) > 1:
        # int list
        if only_contains_int(numstrings):
            nums = [int(num) for num in numstrings]
            return nums
        # float list
        elif only_contains_float(numstrings):
            nums = [float(num) for num in numstrings]
            return nums
        # return a string list
        return numstrings
    return parse_number(text)


def calcBoundingBoxCenter(boxcorners):
    """Calculates the center of a bounding box.

    Args:
      boxcorners(list(list(float): coordinates of the eight cornerpoints of the box

    Returns:
      : mathutils.Vector -- centerpoint of the bounding box

    """
    center = sum((mathutils.Vector(point) for point in boxcorners), mathutils.Vector())
    return center / 8


def sortListsInDict(data, reverse=False):
    """Recursively loops through a dictionary and sorts all lists.

    Args:
      data(dict): data dictionary
      reverse: (Default value = False)

    Returns:
      : dict -- sorted dictionary

    """
    if isinstance(data, list):
        if not data:
            return data

        if isinstance(data[0], dict):
            if all('name' in elem for elem in data):
                return sorted(data, key=lambda k: k['name'], reverse=reverse)
        elif isinstance(data[0], str):
            return sorted(data, reverse=reverse)
        return data
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
      : dict -- dictionary with rounded floats

    """
    epsilon = 10 ** -decimals
    if is_float(data):
        if isinstance(data, str):
            log("Skipping rounding of " + data + " due to its type 'str'", "WARNING")
            return data
        return 0 if abs(data) < epsilon else round(data, decimals)
    elif isinstance(data, list):
        return [roundFloatsInDict(a, decimals) for a in data]
    elif isinstance(data, dict):
        return {key: roundFloatsInDict(value, decimals) for key, value in data.items()}
    else:
        return data


def calculateSum(objects, numeric_prop):
    """Returns sum of *numeric_prop* in *objects*.

    Args:
      objects(list(bpy.types.Object): objects to sum up the property for
      numeric_prop(str): name of the custom property to sum

    Returns:
      : float/int -- sum of the values in the property of the objects

    """
    numsum = 0
    for obj in objects:
        try:
            numsum += obj[numeric_prop]
        except KeyError:
            log(
                "{0} object {1} does not contain '{2}'".format(
                    obj.phobostype, obj.name, numeric_prop
                ),
                "WARNING",
            )
        except TypeError:
            log(
                "Could not add this type to the sum: "
                + str(type(obj[numeric_prop]))
                + " @"
                + nUtils.getObjectName(obj),
                'WARNING',
            )
    return numsum


# TODO is this still needed?
def datetimeFromIso(iso):
    """Accepts a date-time string in ISO format and returns a datetime object.

    Args:
      iso(str): ISO format string for a date and time

    Returns:
      : datetime.datetime -- datetime object from the specified string

    """
    try:
        dtime = datetime(*[int(a) for a in re.split(":|-|T| |\.", iso)])
        return dtime
    except ValueError as error:
        log("Could not convert ISO string: " + str(error), "ERROR")
        return datetime.now()


def distance(objects):
    """Returns the distance between two Blender objects.

    Args:
      objects(list(bpy.types.Object): exactly two Blender objects

    Returns:
      : tuple(float, mathutils.Vector) -- distance and distance vector between the Blender objects

    """
    vector = objects[0].matrix_world.to_translation() - objects[1].matrix_world.to_translation()
    return vector.length, vector


def outerProduct(v, u):
    """Returns a mathutils.Matrix representing the outer product of vectors v and u.

    Args:
      v(mathutils.Vector): first vector
      u(mathutils.Vector): second vector

    Returns:
      : mathutils.Matrix -- outer product of v and u

    """
    lines = []
    for vi in v:
        lines.append([vi * ui for ui in u])
    return mathutils.Matrix(lines)


# TODO is this still needed?
def copyTree(src, dst, symlinks=False, ignore=None):
    """Copies the folder tree from src to dst.
    
    Symlinks and ignore can be specified according to the usage of `shutil.copytree` and
    `shutil.copy2`.

    Args:
      src(str): path to the source folder/item
      dst(str): path to the destination folder/item
      symlinks(bool, optional): whether to copy symlinks to the destination (Default value = False)
      ignore(callable, optional): callable function (see `shutil.copytree`) (Default value = None)

    Returns:

    """
    if not os.path.exists(dst):
        os.makedirs(dst)

    for item in os.listdir(src):
        s_item = os.path.join(src, item)
        d_item = os.path.join(dst, item)
        if os.path.isdir(s_item):
            shutil.copytree(s_item, d_item, symlinks, ignore)
        else:
            shutil.copy2(s_item, d_item)
