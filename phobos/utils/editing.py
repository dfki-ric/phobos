#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.general
    :platform: Unix, Windows, Mac
    :synopsis: This module contains general functions to use in operators and custom scripts

.. moduleauthor:: Kai von Szadowski

Copyright 2017, University of Bremen & DFKI GmbH Robotics Innovation Center

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
import mathutils
import math
from phobos.phoboslog import log
from . import selection as sUtils
from . import naming as nUtils
from . import blender as bUtils
from .. import defs


def addDictionaryToObj(dict, obj, category=None):
    # DOCU add some docstring
    for key, value in dict:
        obj[(category+'/'+key) if category else key] = value


def getCombinedTransform(obj, effectiveparent):
    # DOCU add some docstring
    parent = obj.parent
    matrix = obj.matrix_local
    while parent != effectiveparent and parent is not None:
        matrix = parent.matrix_local * matrix
        parent = parent.parent
    return matrix


def restructureKinematicTree(link):
    """
    Restructures a tree such that the *link* provided becomes the root of the tree. For
    instance, the following tree:
             A
           /  \
          B    C
         / \    \
        D   E    F
    would, using the call restructureKinematicsTree(C), become:
            C
          /  \
         A    F
        /
       B
      / \
     D   E
     Currently, this function ignores all options such as unselected or hidden objects.
    :param link:
    :return:
    """
    root = sUtils.getRoot(link)
    links = [link]
    obj = link

    # gather chain of links ascending the tree
    while not obj.parent == root:
        obj = obj.parent
        if obj.phobostype == 'link':
            links.append(obj)
    links.append(root)

    # unparent all links
    sUtils.selectObjects(links, True)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    for i in range(len(links) - 2):
        parent = links[i]
        child = links[i+1]
        sUtils.selectObjects((parent, child), True, active=0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')


def instantiateSubmodel(submodelname, instancename, size=1.):
    """
    Creates an instance of the submodel specified by the submodelname.

    The instance receives the definitions of the group as it is generated.

    :param submodelname: name of the submodel (Blender group) to create an
        instance of
    :param instancename: name the instance object will receive
    """
    submodel = None
    interfaces = None

    # find the existing group for submodel and interface
    for group in bpy.data.groups:
        # search for namespaced groups with the exact name
        if ':' in group.name and submodelname == group.name:
            submodel = group
        if (group.name.startswith('interfaces:') and
                submodelname.split(':')[1] in group.name):
            interfaces = group

    if not submodel:
        log('Selected submodel is not defined.', 'ERROR')
    if not interfaces:
        log('No interfaces defined for this submodel.', 'INFO')

    # add the submodel and write in data
    bpy.ops.object.group_instance_add(group=submodel.name)
    submodelobj = bpy.context.active_object
    submodelobj.phobostype = 'submodel'
    submodelobj['submodeltype'] = submodel.name.split(':')[0]
    # TODO currently this works only by name binding, we should add links to
    # the group here
    submodelobj['submodelname'] = submodelname
    # copy custom props from group to instance
    for key in submodel.keys():
        submodelobj[key] = submodel[key]
    submodelobj.name = instancename
    submodelobj.empty_draw_size = size

    # add the interfaces if available
    if interfaces:
        bpy.ops.object.group_instance_add(group=interfaces.name)
        bpy.ops.object.duplicates_make_real()
        for obj in bpy.context.selected_objects:
            nUtils.addNamespace(obj, instancename)
            obj.name = obj.name.rsplit('.')[0]
            obj['submodeltype'] = 'interface'
        sUtils.selectObjects(
            objects=[submodelobj] + bpy.context.selected_objects,
            clear=True, active=0)
        bpy.ops.object.parent_set(type='OBJECT')

        # delete empty parent object
        sUtils.selectObjects(objects=[a for a in bpy.context.selected_objects
                                      if a.type == 'EMPTY' and
                                      'interface' in a.name],
                             clear=True, active=0)
        bpy.ops.object.delete(use_global=False)
    return submodelobj


def defineSubmodel(submodelname, submodeltype, version='', objects=None):
    """
    Defines a new submodule group with the specified name and type.

    The group will be named like so:
        'submodeltype:submodelname/version'

    Objects with the phobostype 'interface' (if present) are handled separately
    and put into a respective submodel group (which features the 'interface'
    submodeltype).

    If the version is omitted, the respective part of the name is dropped, too.
    If no object list is provided the objects are derived from selection.
    The submodeltype is also added as dict entry to the group in Blender.

    The selected objects are moved to the respective layer for submodels or
    interfaces.

    :param submodelname: descriptive name of the submodel
    :param submodeltype: type of the submodel (e.g. 'fmu', 'mechanics')
    :param version: a version string (e.g. '1.0', 'dangerous')
    :param objects: the objects which belong to the submodel (None will derive
        objects from the selection)
    :returns: a tuple of the submodelgroup and interfacegroup/None
    """
    if not objects:
        objects = bpy.context.selected_objects

    # split interface from physical objects
    interfaces = [i for i in objects if i.phobostype == 'interface']
    physical_objects = [p for p in objects if p.phobostype != 'interface']

    # make the physical group
    sUtils.selectObjects(physical_objects, True, 0)
    submodelgroupname = submodeltype + ':' + submodelname
    if version != '':
        submodelgroupname += '/' + version
    # TODO what about overwriting groups with same names?
    bpy.ops.group.create(name=submodelgroupname)
    submodelgroup = bpy.data.groups[submodelgroupname]
    submodelgroup['submodeltype'] = submodeltype
    submodelgroup['version'] = version

    modeldefs = defs.definitions['submodeltypes'][submodeltype]

    # copy the definition parameters to the group properties
    for key in modeldefs['definitions']:
        submodelgroup[key] = modeldefs['definitions'][key]

    # move objects to submodel layer
    for obj in physical_objects:
        obj.layers = bUtils.defLayers(defs.layerTypes['submodel'])
    log('Created submodel group ' + submodelname + ' of type "' + submodeltype
        + '".', 'DEBUG')

    interfacegroup = None
    # make the interface group
    if interfaces != []:
        sUtils.selectObjects(interfaces, True, 0)
        interfacegroupname = 'interfaces:' + submodelname
        if version != '':
            interfacegroupname += '/' + version
        # TODO what about overwriting groups with same names?
        bpy.ops.group.create(name=interfacegroupname)
        interfacegroup = bpy.data.groups[interfacegroupname]
        interfacegroup['submodeltype'] = 'interfaces'

        # copy interface definitions from submodel definitions
        for key in modeldefs['interfaces']:
            interfacegroup[key] = modeldefs['interfaces'][key]

        # move objects to interface layer
        for obj in interfaces:
            obj.layers = bUtils.defLayers(defs.layerTypes['interface'])
        log('Created interface group for submodel ' + submodelname + '.',
            'DEBUG')

    for i in interfaces:
        i.show_name = True
    return (submodelgroup, interfacegroup)


def removeSubmodel(submodelname, submodeltype, version='', interfaces=True):
    """
    Removes a submodel definition from the Blender project.
    Returns True or False depending on whether groups have been removed or not.

    :param submodelname: the name of the submodel
    :param submodeltype: the submodeltype of the submodel
    :param version: optional version of the submodel
    :param interfaces: True if interface should also be deleted, else False.
    :returns: True if groups have been removed, else False.
    """
    # build the group name to look for
    submodelgroupname = submodeltype + ':' + submodelname
    if version != '':
        submodelgroupname += '/' + version

    # remove the submodelgroup
    if submodelgroupname in bpy.data.groups:
        bpy.data.groups.remove(bpy.data.groups[submodelgroupname])
        if not interfaces:
            return True

    if interfaces:
        interfacegroupname = 'interfaces:' + submodelname
        if version != '':
            interfacegroupname += '/' + version

        if interfacegroupname in bpy.data.groups:
            bpy.data.groups.remove(bpy.data.groups[interfacegroupname])
            return True
    return False


def toggleInterfaces(interfaces=None, modename='toggle'):
    modedict = {'toggle': 0, 'activate': 1, 'deactivate': 2}
    mode = modedict[modename]
    if not interfaces:
        interfaces = [i for i in bpy.context.selected_objects if i.phobostype == 'interface']
    for i in interfaces:
        if mode == 0:
            i.show_name = not i.show_name
        elif mode == 1:
            i.show_name = True
        elif mode == 2:
            i.show_name = False


def connectInterfaces(parentinterface, childinterface):
    # first check if the interface is child of the root object and if not, restructure the tree
    root = sUtils.getRoot(childinterface)
    parent = childinterface.parent
    if root != parent:
        restructureKinematicTree(parent)
    childsubmodel = childinterface.parent

    # connect the interfaces
    sUtils.selectObjects(objects=[childinterface], clear=True, active=0)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    sUtils.selectObjects(objects=[childinterface, childsubmodel], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    eul = mathutils.Euler((math.radians(180.0), 0.0, math.radians(180.0)), 'XYZ')
    sUtils.selectObjects(objects=[parentinterface, childinterface], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    childinterface.matrix_world = parentinterface.matrix_world * eul.to_matrix().to_4x4()

    # TODO clean this up
    # try:
    #    del childsubmodel['modelname']
    # except KeyError:
    #    pass
    #TODO: re-implement this for MECHANICS models
    # try:
    #     # parent visual and collision objects to new parent
    #     children = sUtils.getImmediateChildren(parent, ['visual', 'collision', 'interface'])
    #     print(children)
    #     sUtils.selectObjects(children, True, 0)
    #     bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    #     print()
    #     sUtils.selectObjects([sUtils.getEffectiveParent(parent, ignore_selection=True)] + children, True, 0)
    #     bpy.ops.object.parent_set(type='BONE_RELATIVE')
    # except (IndexError, AttributeError):
    #     pass  # no objects to re-parent
    parentinterface.show_name = False
    childinterface.show_name = False


def getPropertiesSubset(obj, category=None):
    if not category:
        category = obj.phobostype
    try:
        dict = {key.replace(category+'/', ''): value
                for key, value in obj.items() if key.startswith(category+'/')}
    except KeyError:
        log("Failed filtering properties for category " + category, "ERROR")
    return dict
