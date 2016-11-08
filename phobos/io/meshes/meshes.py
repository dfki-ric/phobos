#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.utils.export
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadkowski, Malte Langosz

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

File smurf.py

Created on 12 Sep 2016
"""

import os
import bpy
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils


def exportObj(path, obj):
    """This function exports a specific object to a chosen path as an .obj

    :param path: The path you want the object export to. *without the filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: .types.Object

    """
    objname = nUtils.getObjectName(obj)
    tmpobjname = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = os.path.join(path, obj.data.name + "." + 'obj')
    bpy.ops.export_scene.obj(filepath=outpath, use_selection=True, use_normals=True, use_materials=False,
                             use_mesh_modifiers=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = tmpobjname


def exportStl(path, obj):
    """This function exports a specific object to a chosen path as a .stl

    :param path: The path you want the object exported to. *without filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object

    """
    objname = nUtils.getObjectName(obj)
    tmpobjname = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = os.path.join(path, obj.data.name + "." + 'stl')
    bpy.ops.export_mesh.stl(filepath=outpath, use_selection=True, use_mesh_modifiers=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = tmpobjname


def exportDae(path, obj):
    """This function exports a specific object to a chosen path as a .dae

    :param path: The path you want the object exported to. *without filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object

    """
    objname = nUtils.getObjectName(obj)
    tmpobjname = obj.name
    obj.name = 'tmp_export_666'  # surely no one will ever name an object like so
    tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    tmpobject.data = obj.data  # copy the mesh here
    outpath = os.path.join(path, obj.data.name + "." + 'dae')
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.wm.collada_export(filepath=outpath, selected=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = tmpobjname
