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
    """Returns list of all objects in the current scene matching phobostype"""
    objlist = []
    for obj in bpy.context.scene.objects:
        if obj.phobostype == phobostype:
            objlist.append(obj)
    return objlist


def getChildren(root):
    """Finds all children for a given root"""
    children = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if getRoot(obj) == root:
            children.append(obj)
    return children


def getImmediateChildren(obj, phobostypes=None):
    """Finds all immediate children for a given object"""
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
    """Finds the root object of a model given one of the model elements is selected or provided"""
    if obj == None:
        for anobj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
            if (anobj.select):
                obj = anobj
    child = obj
    if child == None:
        log("No root object found! Check your object selection", "ERROR")
        return None
    while child.parent != None:
        child = child.parent
    return child


def getRoots():
    """Returns a list of all roots (=objects without parent) present in the scene"""
    roots = []
    for obj in bpy.data.objects:  # TODO: this is not the best list to iterate over (there might be multiple scenes)
        if not obj.parent and obj.phobostype == "link":
            roots.append(obj)

    if roots == []:
        print("Phobos: No root objects found.")
    else:
        print("Phobos: Found", len(roots), "root object(s)", [root.name + "; " for root in roots])
    return roots  # TODO: Should we change this and all other list return values in a tuple or generator expression?


def selectObjects(objects, clear=True, active=-1):
    """Selects all objects provided in list, clears current selection if clear is True
    and sets one of the objects the active objects if a valid index is provided."""

    ##Jan Paul: solution to "context is incorrect" error from
    ##doc/python_api/examples/bpy.ops.py:
    ## check poll() to avoid exception:
    # if bpy.ops.object.mode_set.poll():
    # bpy.ops.object.mode_set(mode='OBJECT')

    if clear:
        bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select = True
    if active >= 0:
        bpy.context.scene.objects.active = objects[active]


def getObjectByName(name):
    objlist = []
    for obj in bpy.data.objects:
        if name == obj.name:
            objlist.append(obj)
        else:
            for type in defs.subtypes:
                nameTag = type + "/name"
                if nameTag in obj and name == obj[nameTag]:
                    objlist.append(obj)
                    break
    return objlist


def selectByName(name):
    selectObjects(getObjectByName(name), True)