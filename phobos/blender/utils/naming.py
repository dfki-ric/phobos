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
Contains the utility functions to rename objects, add/remove namespaces etc.
"""

import re

import bpy


def getUniqueName(newname, names):
    """Generates a new name similar to the Blender internal convention.
    
    This ensures, that the Blender internal limit to object name lengths (63 characters) is matched
    without omitting the number. That way, unwanted naming behaviour is avoided.

    Args:
      newname: desired object name
      names: list of existing names (e.g. bpy.data.objects)

    Returns:
      : str -- new name that is unique in the Blender namespace

    """
    i = 0
    while newname in names:
        numberstr = '.{0:03d}'.format(i)
        newname = newname[: 63 - len(numberstr)] + numberstr
        i += 1
    return newname


def safelyName(obj, name, phobostype=None):
    """Assigns a name to an object in a safe way with regard to the internal
     name handling in Blender.
    
     If no ``phobostype`` is provided or the phobostype is the same as the object itself, the actual
     object is renamed, generating a name that no other object in Blender has, using Blender's own
     naming scheme. This prevents Blender to assign the name and change another object's name that
     previously held that name.

    Args:
      obj(bpy.types.Object): object to rename
      name(str): new name for the object
      phobostype(str, optional): only rename if the specified phobostype is matched (Default value =
    None)

    Returns:
      str: new name of the Blender object

    """
    from phobos.blender.phoboslog import log

    objectname = name
    if not phobostype:
        phobostype = obj.phobostype

    if obj.phobostype == phobostype and obj.name != objectname:
        objectname = getUniqueName(objectname, bpy.data.objects)
        log("Acquired unique name for Blender object: " + objectname, 'DEBUG')
        obj.name = objectname

    return obj.name


def getObjectName(obj, phobostype=None):
    """Returns the name of an object relevant to Phobos.
    An optional *phobostype* parameter can be provided for objects with multiple
    uses, such as link objects (which also contain joint and motor information).
    If no *phobostype* is provided, the phobostype of the object is used.
    The object name itself is returned, stripped of namespaces.

    Args:
      obj(bpy.types.Object): object for which the name is requested
      phobostype: phobostype if relevant (e.g. 'motor') (Default value = None)

    Returns:
      : str -- The object's name

    """
    if obj is None:
        return None
    nametype = phobostype if phobostype else obj.phobostype
    return stripNamespaceFromName(obj.name)


def isValidModelname(name):
    """Returns if a name contains characters other than alphanumeric, '_' and '-'.
    Also, empyt strings are not valid model names.

    Args:
        name (str): potential name for a model

    Returns:
        bool -- True if the name is a valid model name according to convention, False if not.
    """
    return not re.search(r'[^A-Za-z0-9_\-\\]', name) and len(name) > 0


def getModelName(obj):
    """Returns the name of the model encoded by obj, provided that obj is a valid model root.
    
    If obj does not contain a defined 'model/name', the object name is returned with a '*' added
    ad the start, ensuring that the return value will not be treated as a valid model name.

    Args:
      obj(bpy.types.Object): root object of the model

    Returns:
      : str -- modelname of the object or object name with '*' prepended if undefined

    """
    return obj['model/name'] if 'model/name' in obj else '*' + obj.name


def replaceNameElement(prop, old, new):
    """For all selected elements in Blender, replace an *old* part of a string *prop*erty with *new*.

    Args:
      prop: 
      old: 
      new: 

    Returns:

    """
    for obj in bpy.context.selected_objects:
        if prop in obj and obj[prop].find(old) > -1:
            obj[prop] = obj[prop].replace(old, new)


def addNamespaceToName(name, namespace):
    """

    Args:
      name: 
      namespace: 

    Returns:

    """
    return namespace + "::" + name


def addNamespace(obj, namespace=None):
    """Namespaces a given blender object.

    Args:
      obj(bpy.types.Object): The object to namespace.
      namespace: (Default value = None)

    Returns:

    """
    obj.name = safelyName(obj, addNamespaceToName(obj.name, namespace))


def stripNamespaceFromName(name):
    """

    Args:
      name: 

    Returns:

    """
    return name.split('::')[-1]


def removeNamespace(obj):
    """Removes the namespace from an object if present.

    Args:
      obj(bpy.types.Object): The object to remove the namespace from.

    Returns:

    """
    obj.name = safelyName(obj, stripNamespaceFromName(obj.name))


def toggleNamespace(obj, namespace=''):
    """

    Args:
      obj: 
      namespace: (Default value = '')

    Returns:

    """
    if not namespace or namespace + '::' in obj.name:
        removeNamespace(obj)
    else:
        addNamespace(obj, namespace)


def gatherNamespaces(separator='::'):
    """Gathers all existing namespaces.

    Args:
      separator: (Default value = '::')

    Returns:

    """
    namespaces = []
    for obj in bpy.data.objects:
        if separator in obj.name:
            namespaces.append(obj.name.split(separator)[0])
    return namespaces
