#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.selection
    :platform: Unix, Windows, Mac
    :synopsis: This module contains functions to find and select objects

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert, Simon Reichel

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
from phobos.phoboslog import log


def getObjectsByPhobostypes(phobostypes):
    """
    Returns list of all objects in the current scene matching phobostype

    :param phobostypes: the phobostypes to match objects with.
    :type phobostypes: list.
    :return: list - Blender objects.
    """
    return [obj for obj in bpy.context.scene.objects if obj.phobostype in phobostypes]


def getChildren(root, phobostypes=(), selected_only=False, include_hidden=True):
    """
    Finds all (selected or unselected / hidden or unhidden) children of a
    given root object and phobostypes. If phobostypes is not provided, it is ignored.

    :param root: object to start search from.
    :type root: bpy.types.Object.
    :param phobostypes: phobostypes to limit search to.
    :type phobostypes: list of strings.
    :param selected_only: True to find only selected children, else False.
    :type: selected_only: bool.
    :param include_hidden: True to include hidden objects, else False.
    :type: include_hidden: bool.
    :return: list - Blender objects which are children of root.
    """
    return [child for child in bpy.context.scene.objects if getRoot(child) == root and
            (child.phobostype in phobostypes if phobostypes else True) and
            (not child.hide or include_hidden) and
            (child.select or not selected_only)]


def getImmediateChildren(obj, phobostypes=(), selected_only=False, include_hidden=False):
    """
    Finds all immediate children for a given object and phoboytypes.
    If phobostypes is not provided, it is ignored. Search can be limited to
    selected objects and restricted to hidden objects.

    :param obj: object to start search from.
    :type obj: bpy.types.Object.
    :param phobostypes: phobostypes to limit search to.
    :type phobostypes: list of strings.
    :param selected_only: True to find only selected children, else False.
    :type: selected_only: bool.
    :param include_hidden: True to include hidden objects, else False.
    :type: include_hidden: bool.
    :return: list - Blender objects which are immediate children of obj.
    """
    return [child for child in bpy.context.scene.objects if child.parent == obj and
            (child.phobostype in phobostypes if phobostypes else True) and
            (not child.hide or include_hidden) and
            (child.select or not selected_only)]


def getEffectiveParent(obj, ignore_selection=False, include_hidden=False):
    """
    Returns the parent of an object, i.e. the first *link* ascending the
    object tree that is selected, starting from the obj, optionally also excluding
    hidden objects.

    :param obj: object of which to find the parent.
    :type obj: bpy.types.Object.
    :param include_hidden: True to include hidden objects, else False.
    :type: include_hidden: bool.
    :return: bpy.types.Object - the effective parent of the obj.
    """
    parent = obj.parent
    while (parent and ((parent.hide and not include_hidden) or
            (not parent.select and bpy.data.worlds[0].phobosexportsettings.selectedOnly
             and not ignore_selection)
             or parent.phobostype != 'link')):
        parent = parent.parent
    return parent


def getRoot(obj=None):
    """
    Returns the root object of a model the Blender object obj or, if obj is
    not provided, the active object is part of, traversing up the tree.
    If no such object is found, returns None.

    :param obj: The object to find the root for.
    :type obj: bpy.types.Object.
    :return: bpy.types.Object - The root object.
    """
    obj = bpy.context.active_object if obj is None else obj
    if obj is None:
        log("No root object found! Check your object selection.", "ERROR")
        return None
    else:
        child = obj
        while child.parent and not isRoot(child):
            child = child.parent
        return child


def getRoots():
    """
    Returns a list of all of the current scene's root links, i.e. links containing a model
    name or entity name.

    :return: list - all root links.
    """
    roots = [obj for obj in bpy.context.scene.objects if isRoot(obj)]
    if roots is None:
        log("Phobos: No root objects found.", "WARNING", "getRoots")
    else:
        log("Phobos: Found " + str(len(roots)) + " root object(s): " + str(roots), "DEBUG", "getRoots")
    return roots  # TODO: Should we change this and all other list return values in a tuple or generator expression?


def isRoot(obj):
    """
    Returns whether or not the object passed to obj is a Phobos model root.

    :param obj: The object for which model root status is tested.
    :type obj: bpy.types.Object.
    :return: bool - True if obj is Phobos model root, else False.
    """
    return None if obj is None else ('modelname' in obj and obj.phobostype in ['link', 'assembly']
                                     and obj.parent is None)


def isEntity(obj):
    """
    Returns whether or not the opject passed is an Phobos entity.

    :param obj: The object for which entity status is tested.
    :type obj: bpy.types.Object.
    :return: bool - True if obj is an entity, else False.
    """
    return None if obj is None else ('entity/type' in obj and 'entity/name' in obj)


def selectObjects(objects, clear=True, active=-1):
    """
    Selects all objects provided in list, clears current selection if clear is True
    and sets one of the objects the active objects if a valid index is provided.

    :param objects: the objects to be selected.
    :type objects: list of bpy.types.Object.
    :param clear: True to clear current selected objects before selection, else False.
    :type clear: bool.
    :param active: index of the object to set active.
    :type active: int.
    :return: None.
    """
    # if no object is active, object mode can't be toggled
    if bpy.context.scene.objects.active:
        bpy.ops.object.mode_set(mode='OBJECT')
    if clear:
        # TODO delete me?
        # bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select = True
    if active >= 0:
        bpy.context.scene.objects.active = objects[active]


def getObjectByName(name):
    """
    Gets blender object by its name (blender objects name or subtypes name).

    :param name: The exact object name to find.
    :type name: str.
    :return: list - all found objects.
    """
    objlist = []
    for obj in bpy.context.scene.objects:
        if name == obj.name:
            objlist.append(obj)
        else:
            for subtype in defs.subtypes:
                nametag = subtype + "/name"
                if nametag in obj and name == obj[nametag]:
                    objlist.append(obj)
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
    :return: list - all matching objects.
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
    :return: bpy.types.Object - the matching object.
    """
    name_tag = phobostype + "/name"
    for obj in bpy.data.objects:
        if name_tag in obj and name == obj[name_tag]:
            return obj
    log("No object of type " + phobostype + " with name " + name + " found.",
        "WARNING", "getObjectByNameAndType")
    return None


def selectByName(name, match_case=False):
    """
    Uses getObjectsByPattern to select the found objects.

    :param pattern: The pattern to search for.
    :type pattern: str.
    :param match_case: Indicate whether to match the object names' case to the pattern.
    :type match_case: bool.
    :return: None.
    """
    # TODO delete me?
    # selectObjects(getObjectByName(name), True)
    selectObjects(getObjectsByPattern(name, match_case), True)


def getSelectedObjects():
    """
    Returns a generator of all selected objects independent of bpy.context.

    # DOCU fill this in
    :return:
    """
    return (obj for obj in bpy.context.scene.objects if obj.select)


def getObjectsByProperty(property, value):
    # DOCU add some docstring
    candidate = None
    for obj in bpy.data.objects:
        if property in obj and obj[property] == value:
            candidate = obj
            break
    return candidate
