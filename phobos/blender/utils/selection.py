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
Contains the utility functions for selecting objects in Blender based on different criteria.
"""

import bpy

from ..phoboslog import log
from . import naming as nUtils


def getLeaves(roots, objects=[]):
    """Returns the links representating the leaves of the spanning tree starting with an object
    inside the model spanning tree.

    Args:
      root(list of bpy.types.Object): Root objects from where to start the search from
      objects(list, optional): List of objects to which the search is restricted. (Default value = [])
      roots: 

    Returns:
      list: List of the leaves of the kinematic spanning tree.

    """
    leaves = []

    if isinstance(roots, list):
        for root in roots:
            leaves += getLeaves(root, objects=objects)

    else:
        if roots.phobostype != 'link':
            roots = getEffectiveParent(roots, objectlist=objects)

        candidates = getImmediateChildren(roots, phobostypes=('link'))

        if objects and candidates:
            candidates = [candidate for candidate in candidates if candidate in objects]

        if candidates:
            leaves += getLeaves(candidates, objects=objects)
        else:
            leaves.append(roots)

    if objects:
        leaves = [leave for leave in leaves if leave in objects]

    # Remove possible doubles
    outputs = []

    for leave in leaves:
        if leave not in outputs:
            outputs.append(leave)

    return outputs


def getObjectsByPhobostypes(phobostypes):
    """Returns list of all objects in the current scene matching phobostype

    Args:
      phobostypes(list): the phobostypes to match objects with.

    Returns:
      : list - Blender objects.

    """
    return [obj for obj in bpy.context.scene.objects if obj.phobostype in phobostypes]


def getChildren(root, phobostypes=(), selected_only=False, include_hidden=True):
    """Finds all (selected or unselected / hidden or unhidden) children of a
    given root object and phobostypes. If phobostypes is not provided, it is ignored.

    Args:
      root(bpy.types.Object): object to start search from.
      phobostypes(list of strings, optional): phobostypes to limit search to. (Default value = ()
      selected_only(bool., optional): True to find only selected children, else False. (Default value = False)
      include_hidden(bool., optional): True to include hidden objects, else False. (Default value = True)

    Returns:
      list: Blender objects which are children of root.

    """
    return [
        child
        for child in bpy.context.scene.objects
        if getRoot(child) == root
        and (child.phobostype in phobostypes if phobostypes else True)
        and (not child.hide_viewport or include_hidden)
        and (child.select_get() or not selected_only)
    ]


def getImmediateChildren(obj, phobostypes=(), selected_only=False, include_hidden=False):
    """Returns all immediate children for a given object and phobostypes (if provided).
    Search can be limited to selected objects and non-hidden objects.

    Args:
      obj(bpy.types.Object): object to start search from.
      phobostypes(list of strings, optional): phobostypes to limit search to. (Default value = ()
      selected_only(bool., optional): True to find only selected children, else False. (Default value = False)
      include_hidden(bool., optional): True to include hidden objects, else False. (Default value = False)

    Returns:
      : list - Blender objects which are immediate children of obj.

    """
    return [
        child
        for child in obj.children
        if (child.phobostype in phobostypes if phobostypes else True)
        and (not child.hide_viewport or include_hidden)
        and (child.select_get() or not selected_only)
    ]


def getRecursiveChildren(
    obj, recursion_depth=None, phobostypes=(), selected_only=False, include_hidden=False
):
    """Returns all children for a given object and phobostypes (if provided) within the given recursion depth.
    Search can be limited to selected objects and non-hidden objects.

    Args:
      obj(bpy.types.Object): object to start search from.
      recursion_depth(int, optional): Depth of the recursion, default is all levels (Default value = 0)
      phobostypes(list of strings, optional): phobostypes to limit search to. (Default value = ()
      selected_only(bool., optional): True to find only selected children, else False. (Default value = False)
      include_hidden(bool., optional): True to include hidden objects, else False. (Default value = False)

    Returns:
      list: Blender objects which are children of obj within recursion depth.

    """
    # If no recursion, than find current children
    children = getImmediateChildren(obj, phobostypes, selected_only, include_hidden)

    if recursion_depth is None or recursion_depth > 0 and children:
        new_children = []
        for child in children:
            new_children += getRecursiveChildren(
                child, recursion_depth - 1 if recursion_depth is not None else None, phobostypes, selected_only, include_hidden
            )
        children += new_children

    return children


def getEffectiveParent(obj, ignore_selection=True, include_hidden=True, objectlist=[]):
    """Returns the parent of an object, i.e. the first *link* ascending the
    object tree that is selected, starting from the obj, optionally also excluding
    hidden objects.

    Args:
      obj(bpy.types.Object): object of which to find the parent.
      include_hidden(bool, optional): True to include hidden objects, else False. (Default value = False)
      ignore_selection(bool, optional): whether or not to use the current selection as limitation (Default value = False)
      objectlist: list of bpy.types.Object to which possible parents are restricted (Default value = [])

    Returns:

    """
    if not objectlist:
        objectlist = list(bpy.data.objects)

    parent = obj.parent
    while (
        parent
        and parent in objectlist
        and parent.phobostype != "link"
        and (
            (parent.hide_viewport and not include_hidden)
            or (
                not parent.select_get()
                and bpy.context.scene.phobosexportsettings.selectedOnly
                and not ignore_selection
            )
        )
    ):
        parent = parent.parent
    return parent


def getRoot(obj=None, verbose=True):
    """Returns the root object of a model the Blender object obj or, if obj is
    not provided, the active object is part of, traversing up the tree.
    If no such object is found, returns None.

    Args:
      obj(bpy.types.Object, optional): The object to find the root for. (Default value = None)

    Returns:
      : bpy.types.Object - The root object.

    """
    obj = bpy.context.active_object if obj is None else obj
    if obj is None:
        if verbose:
            log("No root object found! Check your object selection.", "ERROR")
        return None
    child = obj
    while child.parent and not isRoot(child):
        child = child.parent
    return child


def getRoots(scene=None):
    """Returns a list of all of the current/specified scene's root objects.

    Args:
      scene(bpy.types.Scene, optional): the scene of which to find the root objects (Default value = None)

    Returns:
      : list(bpy.types.Object) -- root objects of the scene

    """
    if not scene:
        scene = bpy.context.scene

    roots = [obj for obj in scene.objects if isRoot(obj, scene=scene)]
    if roots is None:
        log("No root objects found in scene {}.".format(scene), 'WARNING')
    else:
        rootnames = ', '.join((root.name for root in roots))
        log(
            "Found {} root object{} in scene {}: {}".format(
                len(roots), 's' if len(roots) > 1 else '', scene, rootnames
            ),
            'DEBUG',
        )
    return roots


def isRoot(obj, scene=None):
    """Returns whether or not the object passed to obj is a Phobos model root.

    Args:
      obj(bpy.types.Object): The object for which model root status is tested.
      scene: (Default value = None)

    Returns:

    """
    rootdefinition = obj is not None and obj.phobostype == "link"
    parentless = not obj.parent or (scene is not None and obj.parent.name not in scene.objects)

    return rootdefinition and parentless


def getRootsOfSelection():
    """TODO Missing documentation"""
    return list(set([getRoot(obj) for obj in bpy.context.selected_objects]))


def isEntity(obj):
    """Returns whether or not the opject passed is an Phobos entity.

    Args:
      obj(bpy.types.Object): The object for which entity status is tested.

    Returns:

    """
    return obj is not None and 'entity/type' in obj and 'entity/name' in obj


def getModelRoot(modelname):
    roots = getRoots()
    for root in roots:
        if nUtils.getModelName(root) == modelname:
            return root


def selectModel(modelname):
    selection = []
    if modelname:
        log("phobos: Selecting model" + modelname, "INFO")
        roots = getRoots()
        for root in roots:
            if nUtils.getModelName(root) == modelname:
                selection = getChildren(root)
        if not selection:
            log("phobos: No proper modelname given", "ERROR")
    else:
        log("No model name provided, deriving from selection...", "INFO")
        roots = set()
        for obj in bpy.context.selected_objects:
            roots.add(getRoot(obj))
        for root in list(roots):
            selection.extend(getChildren(root))
    selectObjects(list(selection), True)


def selectObjects(objects, clear=True, active=-1):
    """Selects all objects provided in list, clears current selection if clear is True
    and sets one of the objects the active objects if a valid index is provided.

    Args:
      objects(iterable of bpy.types.Object): the objects to be selected.
      clear(bool, optional): clear current selection? (Default value = True)
      active(int, optional): index of the object to set active. (Default value = -1)

    Returns:
      : None.

    """
    # if no object is active, object mode can't be toggled
    if bpy.context.view_layer.objects.active:
        bpy.ops.object.mode_set(mode='OBJECT')
    if clear:
        bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    if active >= 0:
        bpy.context.view_layer.objects.active = objects[active]


storedSelection = []
storedActive = -1


def storeSelectedObjects():
    global storedSelection, storedActive
    storedSelection = list(bpy.context.selected_objects)
    try:
        storedActive = storedSelection.index(bpy.context.view_layer.objects.active)
    except ValueError:
        storedActive = -1
    print("Storage:")
    print(storedSelection)
    print(storedActive)


def restoreSelectedObjects():
    global storedSelection, storedActive
    print("Retrieve:")
    print(storedSelection)
    print(storedActive)
    selectObjects(storedSelection, active=storedActive)


def getObjectByName(name, phobostypes=()):
    """Returns list of objects that either have a specific *name* or contain a custom
    name property with that name.
    
    As the function returns either an empty list, a unique object or a list of objects,
    it is possible to test for uniqueness of the result by calling `isinstance(result, list)`.

    Args:
      name(str): The exact object name to find.
      phobostypes: (Default value = ())

    Returns:
      : bpy.types.Object or list - one or list of objects matching name

    """
    found = None
    searchobjs = [
        obj for obj in bpy.context.scene.objects if obj.phobostype in phobostypes or not phobostypes
    ]
    for obj in searchobjs:
        # the link/name handling is only for backwards compatibility
        if name == obj.name or obj.get("joint/name", None) == name or obj.get("link/name", None) == name:
            found = obj
            break
    return found


def getObjectsByPattern(pattern, match_case=False):
    """Return a list of objects in the scene that match a name pattern. The pattern
    may match either the object's actual name or the value of the 'phobostype/name'
    property.

    Args:
      pattern: str
      match_case: bool (Default value = False)

    Returns:
      list - all matching objects.

    """
    objlist = []
    for obj in bpy.data.objects:
        for key in obj.keys():
            if key.endswith('/name'):
                objname = obj[key]
                if (match_case and pattern in objname) or (
                    not match_case and pattern.lower() in objname.lower()
                ):
                    objlist.append(obj)
        if (match_case and pattern in obj.name) or (
            not match_case and pattern.lower() in obj.name.lower()
        ):
            objlist.append(obj)
    return objlist


def getObjectByNameAndType(name, phobostype):
    """Find an object with a specified phobostype and having the property
    "phobostype/'name' == name".

    Args:
      name(str): The name to search for.
      phobostype(str): The phobostype to search for.

    Returns:
      : bpy.types.Object - the matching object.

    """
    # FIXME: make this API-compatible with geObjectByName
    name_tag = phobostype + "/name"
    for obj in bpy.data.objects:
        if name_tag in obj and name == obj[name_tag]:
            return obj
    log("No object of type " + phobostype + " with name " + name + " found.", "WARNING")
    return None


def selectByName(name, match_case=False, exact=False):
    """Uses getObjectsByPattern to select the found objects.

    Args:
      name(str): The name (pattern) to search for.
      match_case(bool, optional): Indicate whether to match the object names' case to the pattern. (Default value = False)
      exact(bool, optional): whether to search for exact string or not (Default value = False)

    Returns:
      : None.

    """
    if exact:
        obj = getObjectByName(name)
        selectObjects(object if isinstance(obj, list) else [obj], True)
    else:
        selectObjects(getObjectsByPattern(name, match_case), True)


def getObjectByProperty(property, value):
    """Returns the first object found in the .blend file data with matching property and value

    Args:
      property: 
      value: 

    Returns:

    """
    candidate = None
    for obj in bpy.data.objects:
        if property in obj and obj[property] == value:
            candidate = obj
            break
    return candidate


def getSubmechanismRootForJoint(jointobj):
    """

    Args:
      jointobj: 

    Returns:

    """
    for root in getSubmechanismRoots():
        if jointobj in root['submechanism/spanningtree']:
            return root
