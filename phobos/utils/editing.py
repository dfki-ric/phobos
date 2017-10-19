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


def instantiateAssembly(assemblyname, instancename, version='1.0', as_assembly=True):
    assembly = None
    interfaces = None
    for group in bpy.data.groups:
        if group.name.startswith(assemblyname):
            if group.name.endswith('interfaces'):
                interfaces = group
            else:
                assembly = group
    if not assembly or not interfaces:
        raise RuntimeError('Assembly and/or interfaces templates do not exist.')
    bpy.ops.object.group_instance_add(group=assembly.name)
    assemblyobj = bpy.context.active_object
    assemblyobj.phobostype = 'assembly'
    if as_assembly:
        assemblyobj['assemblyname'] = assemblyname
    else:
        assemblyobj['modelname'] = assemblyname
    assemblyobj['version'] = version
    assemblyobj.name = instancename
    bpy.ops.object.group_instance_add(group=interfaces.name)
    #interfaceobj = bpy.context.active_object
    bpy.ops.object.duplicates_make_real()
    sUtils.selectObjects(objects=[assemblyobj]+bpy.context.selected_objects, clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    sUtils.selectObjects(objects=[a for a in bpy.context.selected_objects
                                  if a.type == 'EMPTY' and a.name.endswith('interfaces')],
                         clear=True, active=0)
    bpy.ops.object.delete(use_global=False)


def defineAssembly(assemblyname, version='', objects=None):
    if not objects:
        objects = bpy.context.selected_objects
    interfaces = [i for i in objects if i.phobostype == 'interface']
    physical_objects = [p for p in objects if p.phobostype != 'interface']
    sUtils.selectObjects(physical_objects, True, 0)
    bpy.ops.group.create(name='assembly:' + assemblyname + '/' + version)
    sUtils.selectObjects(interfaces, True, 0)
    bpy.ops.group.create(name='assembly:' + assemblyname + '/' + version + '/interfaces')
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
    childassembly = childinterface.parent
    sUtils.selectObjects(objects=[childinterface], clear=True, active=0)
    bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
    sUtils.selectObjects(objects=[childinterface, childassembly], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    eul = mathutils.Euler((math.radians(180.0), 0.0, math.radians(180.0)), 'XYZ')
    sUtils.selectObjects(objects=[parentinterface, childinterface], clear=True, active=0)
    bpy.ops.object.parent_set(type='OBJECT')
    childinterface.matrix_world = parentinterface.matrix_world * eul.to_matrix().to_4x4()
    try:
        del childassembly['modelname']
    except KeyError:
        pass


def getPropertiesSubset(obj, category=None):
    if not category:
        category = obj.phobostype
    try:
        dict = {key.replace(category+'/', ''): value
                for key, value in obj.items() if key.startswith(category+'/')}
    except KeyError:
        log("Failed filtering properties for category " + category, "ERROR")
    return dict
