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
from phobos.phoboslog import log


def exportMesh(obj, path, meshtype):
    # DOCU add some docstring
    objname = nUtils.getObjectName(obj)
    tmpobjname = obj.name
    # OPT: surely no one will ever name an object like so, better solution?
    obj.name = 'tmp_export_666'
    tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    # copy the mesh here
    tmpobject.data = obj.data
    outpath = os.path.join(path, obj.data.name + "." + meshtype)
    if meshtype == 'obj':
        bpy.ops.export_scene.obj(filepath=outpath, use_selection=True, use_normals=True, use_materials=False,
                                 use_mesh_modifiers=True)
    elif meshtype == 'stl':
        bpy.ops.export_mesh.stl(filepath=outpath, use_selection=True, use_mesh_modifiers=True)
    elif meshtype == 'dae':
        bpy.ops.wm.collada_export(filepath=outpath, selected=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = tmpobjname


def importMesh(filepath, meshtype):
    # DOCU add some docstring
    # tag all objects
    for obj in bpy.data.objects:
        obj['phobosTag'] = True

    # import mesh
    try:
        mesh_type_dict[meshtype]['import'](filepath)
    except KeyError:
        log('Unknown mesh type: ' + meshtype, 'ERROR')

    # find the newly imported obj
    newgeom = None
    for obj in bpy.data.objects:
        if 'phobosTag' not in obj:
            newgeom = obj

    # with obj file import, blender only turns the object, not the vertices,
    # leaving a rotation in the matrix_basis, which we here get rid of
    if meshtype == 'obj':
        bpy.ops.object.select_all(action='DESELECT')
        newgeom.select = True
        bpy.ops.object.transform_apply(rotation=True)

    # clean the tag
    for obj in bpy.data.objects:
        if 'phobosTag' in obj:
            del obj['phobosTag']

    return newgeom


def importObj(filepath):
    # DOCU add some docstring
    bpy.ops.import_scene.obj(filepath=filepath)


def importStl(filepath):
    # DOCU add some docstring
    bpy.ops.import_mesh.stl(filepath=filepath)


def importDae(filepath):
    # DOCU add some docstring
    bpy.ops.wm.collada_import(filepath=filepath)


def exportObj(obj, path):
    """This function exports a specific object to a chosen path as an .obj

    :param path: The path you want the object export to. *without the filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: .types.Object

    """
    exportMesh(obj, path, 'obj')


def exportStl(obj, path):
    """This function exports a specific object to a chosen path as a .stl

    :param path: The path you want the object exported to. *without filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object

    """
    exportMesh(obj, path, 'stl')


def exportDae(obj, path):
    """This function exports a specific object to a chosen path as a .dae

    :param path: The path you want the object exported to. *without filename!*
    :type path: str
    :param obj: The blender object you want to export.
    :type obj: bpy.types.Object

    """
    exportMesh(obj, path, 'dae')


# registering mesh types with Phobos
mesh_type_dict = {'obj': {'export': exportObj,
                          'import': importObj,
                          'extensions': ('obj',)},
                  'stl': {'export': exportStl,
                          'import': importStl,
                          'extensions': ('stl',)},
                  'dae': {'export': exportDae,
                          'import': importDae,
                          'extensions': ('dae',)}
                  }
