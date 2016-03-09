#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.naming
    :platform: Unix, Windows, Mac
    :synopsis: This module contains functions to name objects

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

import bpy
from phobos.logging import log
import phobos.defs as defs
import phobos.utils.selection as selection


def getObjectName(obj, phobostype=None):
    """This function returns the name for an object depending on its phobostype.
    For links and objects lacking a '*phobostype*/name' property, the object's
    name is used instead, cleaned of namespaces.

    :param obj: The object you want the name for.
    :type obj: bpy.types.Object
    :param phobostype: The phobostype you want this objects name for.
    :return: str -- The objects name.

    """
    if obj is None:
        return None
    nametype = phobostype if phobostype else obj.phobostype
    if nametype != 'link' and nametype + "/name" in obj:
        return obj[nametype + "/name"]
    else:
        return obj.name.split(':')[-1]


def replaceNameElement(prop, old, new):
    """For all selected elements in Blender, replace an *old* part of a string *prop*erty with *new*.

    """
    for obj in bpy.context.selected_objects:
        if prop in obj and obj[prop].find(old) > -1:
            obj[prop] = obj[prop].replace(old, new)


def addNamespace(obj):
    """This function namespaces a given blender object.

    :param obj: The object to namespace.
    :type obj: bpy.types.Object

    """
    types = defs.subtypes
    name = obj.name
    root = selection.getRoot(obj)
    namespace = root["entity/name"] if root != None and "entity/name" in root else None
    if not namespace:
        log("The obj " + getObjectName(obj) + "has no namespace to append to. Aborting.", "ERROR")
        return
    obj.name = namespace + "::" + name
    for pType in types:
        typeTag = pType + "/type"
        nameTag = pType + "/name"
        if (typeTag in obj or ("phobostype" in obj and obj.phobostype == pType)) and nameTag not in obj:
            obj[nameTag] = name


def removeNamespace(obj):
    """This function removes the namespace from an object if present.

    :param obj: The object to remove the namespace from.
    :type obj: bpy.types.Object

    """
    types = defs.subtypes
    name = obj.name.split("::")[-1]
    obj.name = name
    for pType in types:
        nameTag = pType + "/name"
        if nameTag in obj and obj[nameTag] == name:
            del obj[nameTag]


def namesAreExplicit(nameset, objnames):
    """This function checks whether two sets of names have equal names.

    """
    return len(nameset.intersection(objnames)) == 0
