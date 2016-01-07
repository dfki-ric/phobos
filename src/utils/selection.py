#!/usr/bin/python

"""
.. module:: phobos.utils.selection
    :platform: Unix, Windows, Mac
    :synopsis: This module contains functions to find and select objects

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
import phobos.defs as defs
from phobos.logging import log


def returnObjectList(phobostype):
    """Returns list of all objects in the current scene matching phobostype

    """
    objlist = []
    for obj in bpy.context.scene.objects:
        if obj.phobostype == phobostype:
            objlist.append(obj)
    return objlist


def getChildren(root):
    """Finds all children for a given root

    """
    children = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if getRoot(obj) == root:
            children.append(obj)
    return children


def getImmediateChildren(obj, phobostypes=None):
    """Finds all immediate children for a given object

    """
    children = []
    for child in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if child.parent == obj:
            if phobostypes is not None:
                if child.phobostype in phobostypes:
                    children.append(child)
            else:
                children.append(child)
    return children


def getRoot(obj=None):
    """
    Find the root of an object, i.e. the first going up the tree containing a
    model name or entity name. If there is no such object up the tree, the
    tree's top-most object is returned.
    If no object is given, find root of the active object. If there is no
    active object, throw an error.

    :param obj: The object to find the root for.
    :type obj: bpy.types.Object.
    :return: The root object.
    """
    if not obj:
        for anobj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
            if bpy.context.scene.objects.active == anobj:
                obj = anobj
                break
        else:
            log("No root object found! Check your object selection", "ERROR")
            return None
    child = obj
    while child.parent and not ('modelname' in child or 'entityname' in child):
        child = child.parent
    return child


def getRoots():
    """
    Find all of the scene's root links, i.e. links containing a model name or
    entity name.

    :return: List of all root links.
    """
    roots = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if ('modelname' in obj or 'entityname' in obj) and obj.phobostype == "link":
            roots.append(obj)

    if not roots:
        log("Phobos: No root objects found.", "WARNING", "getRoots")
    else:
        log("Phobos: Found " + str(len(roots)) + " root object(s)", "INFO", "getRoots")
    return roots  # TODO: Should we change this and all other list return values in a tuple or generator expression?


def selectObjects(objects, clear=True, active=-1):
    """Selects all objects provided in list, clears current selection if clear is True
    and sets one of the objects the active objects if a valid index is provided.

    """

    ##Jan Paul: solution to "context is incorrect" error from
    ##doc/python_api/examples/bpy.ops.py:
    ## check poll() to avoid exception:
    # if bpy.ops.object.mode_set.poll():
    #     bpy.ops.object.mode_set(mode='OBJECT')

    if clear:
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select = True
    if active >= 0:
        bpy.context.scene.objects.active = objects[active]


def getObjectByName(name):
    """Gets blender object by its name (blender objects name or subtypes name).

    :param name: The exact objects name to find.
    :return: list - containing all found objects.

    """
    objlist = []
    for obj in bpy.data.objects:
        if name == obj.name:
            objlist.append(obj)
        else:
            for type in defs.subtypes:
                nameTag = type + "/name"
                if nameTag in obj and name == obj[nameTag]:
                    objlist.append(obj)
                    # 'break' does not seem to make sense here, as there may be
                    # objects from different types with the wanted name
                    #break
    return objlist


def getObjectsByPattern(pattern, match_case=False):
    """
    Find objects in the scene that match a name pattern. The pattern may match
    either the object's actual name or the value of the 'phobostype/name'
    property.

    :param pattern: The pattern to search for.
    :type pattern: str.
    :param match_case: Indicate whether to match the object names' case to the pattern.
    :type match_case: bool.
    :return: List containing the matching objects.
    """
    obj_list = []
    for obj in bpy.data.objects:
        for subtype in defs.subtypes:
            name_tag = subtype + '/name'
            if name_tag in obj:
                obj_name = obj[name_tag]
                if (match_case and pattern in obj_name) \
                        or (not match_case and pattern.lower() in obj_name.lower()):
                    obj_list.append(obj)
        if (match_case and pattern in obj.name) \
                or (not match_case and pattern.lower() in obj.name.lower()):
            obj_list.append(obj)
    return obj_list


def getObjectByNameAndType(name, phobostype):
    """
    Find an object with a specified phobostype and having the property
    "phobostype/'name' == name".

    :param name: The name to search for.
    :type name: str.
    :param phobostype: The phobostype to search for.
    :type phobostype: str.
    :return: Matching object.
    """
    name_tag = phobostype + "/name"
    for obj in bpy.data.objects:
        if name_tag in obj and name == obj[name_tag]:
            return obj
    log("No object of type " + phobostype + " with name " + name + " found.", "WARNING")
    return None


def selectByName(name, match_case=False):
    """Uses getObjectByName to select the found objects.

    """
    #selectObjects(getObjectByName(name), True)
    selectObjects(getObjectsByPattern(name, match_case), True)
