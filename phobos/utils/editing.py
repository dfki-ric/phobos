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
from . import selection as sUtils
from . import naming as nUtils
from phobos.phoboslog import log


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
    i = 0
    for l in range(len(links)-1):
        parent = links[i]
        child = links[i+1]
        sUtils.selectObjects((parent, child), True, active=0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        i += 1


def instantiateAssembly(assemblyname, instancename, version='1.0', size=1.):
    assembly = None
    interfaces = None

    # find the existing groups with assemblies and interfaces
    for group in bpy.data.groups:
        if group.name.startswith('assembly') and assemblyname in group.name:
            assembly = group
        if group.name.startswith('interfaces') and assemblyname in group.name:
            interfaces = group

    if not assembly or not interfaces:
        raise RuntimeError('Assembly and/or interfaces templates do not exist.')
    # add the assembly and write in data
    bpy.ops.object.group_instance_add(group=assembly.name)
    assemblyobj = bpy.context.active_object
    assemblyobj.phobostype = 'assembly'
    assemblyobj['assemblyname'] = assemblyname
    assemblyobj['version'] = version
    assemblyobj.name = instancename
    assemblyobj.empty_draw_size = size
    # add the interfaces, make them real and get rid of parent empty object
    bpy.ops.object.group_instance_add(group=interfaces.name)
    #interfaceobj = bpy.context.active_object
    bpy.ops.object.duplicates_make_real()
    for obj in bpy.context.selected_objects:
        nUtils.addNamespace(obj, instancename)
        obj.name = obj.name.rsplit('.')[0]
    sUtils.selectObjects(objects=[assemblyobj]+bpy.context.selected_objects, clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    sUtils.selectObjects(objects=[a for a in bpy.context.selected_objects
                                  if a.type == 'EMPTY' and 'interface' in a.name],
                         clear=True, active=0)
    bpy.ops.object.delete(use_global=False)
    return assemblyobj


def defineAssembly(assemblyname, version='', objects=None):
    if not objects:
        objects = bpy.context.selected_objects
    interfaces = [i for i in objects if i.phobostype == 'interface']
    physical_objects = [p for p in objects if p.phobostype != 'interface']
    sUtils.selectObjects(physical_objects, True, 0)
    bpy.ops.group.create(name='assembly:' + assemblyname + '/' + version)
    sUtils.selectObjects(interfaces, True, 0)
    bpy.ops.group.create(name='interfaces:' + assemblyname + '/' + version)
    for i in interfaces:
        i.show_name = True
    # i = 0
    # while objects[i].parent in objects:
    #     i += 1
    # if objects[i].parent and objects[i].parent in objects:
    #     log()


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
    childassembly = childinterface.parent

    # connect the interfaces
    sUtils.selectObjects(objects=[childinterface], clear=True, active=0)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    sUtils.selectObjects(objects=[childinterface, childassembly], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    eul = mathutils.Euler((math.radians(180.0), 0.0, math.radians(180.0)), 'XYZ')
    sUtils.selectObjects(objects=[parentinterface, childinterface], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    childinterface.matrix_world = parentinterface.matrix_world * eul.to_matrix().to_4x4()
    #try:
    #    del childassembly['modelname']
    #except KeyError:
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
