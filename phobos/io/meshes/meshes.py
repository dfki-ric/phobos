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
Contains the functions for the mesh entity.
"""

import os
import bpy
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
from phobos.phoboslog import log


def exportMesh(obj, path, meshtype):
    """
    Exports the specified objects' mesh into the specified path.
    The meshtype specifies the file format for the exported mesh.

    Args:
      obj: bpy.types.object
      path: path
      meshtype: 'obj', 'stl' or 'dae'

    Returns:
        None
    """
    import phobos.utils.io as ioUtils

    objname = nUtils.getObjectName(obj)

    # TODO add a mesh flatten option, to merge meshgroups/link visuals into a single mesh
    if obj.type == 'EMPTY':
        meshname = objname
        meshgroup = True
    else:
        meshgroup = False
        meshname = obj.data.name

    # TODO: why did the old code copy the mesh data into a temporary object?
    # I guess, because we want the mesh scale to be 1, but let's keep this as a reminder:
    # tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    # tmpobject.data = obj.data

    # save scale of object temporarily
    tmpscale = obj.scale
    obj.scale = (1, 1, 1)
    obj.select = True

    # for obj and stl we need to flatten the meshgroup
    if meshgroup and meshtype in ['obj', 'stl']:
        newobj = bUtils.joinMeshes(
            [ob for ob in obj.children if ob.phobostype in ['visual', 'collision']], copy=True)
        obj.select = False
        tmpname = obj.name
        obj.name = nUtils.getUniqueName('tmp', bpy.data.objects)
        newobj.name = tmpname
        newobj.select = True

    outpath = os.path.join(path, meshname + "." + meshtype)
    if meshtype == 'obj':

        # TODO flatten meshgroup objects into single mesh
        axis_forward = bpy.context.scene.phobosexportsettings.obj_axis_forward
        axis_up = bpy.context.scene.phobosexportsettings.obj_axis_up
        bpy.ops.export_scene.obj(
            filepath=outpath,
            use_selection=True,
            use_normals=True,
            use_materials=False,
            use_mesh_modifiers=True,
            axis_forward=axis_forward,
            axis_up=axis_up,
        )
    elif meshtype == 'stl':
        bpy.ops.export_mesh.stl(
            filepath=outpath,
            use_selection=True,
            use_mesh_modifiers=True
        )
        # TODO flatten meshgroup objects into single mesh
    elif meshtype == 'dae':
        bpy.ops.wm.collada_export(
            filepath=outpath,
            selected=True,
            include_children=meshgroup
        )

    # clean up temporary objects from mesh flattening
    if meshgroup and meshtype in ['obj', 'stl']:
        bpy.ops.object.select_all(action='DESELECT')
        newobj.select = True
        bpy.ops.object.delete()
        obj.name = tmpname

    # reapply object scale
    obj.scale = tmpscale


def importMesh(filepath, meshtype):
    """

    Args:
      filepath: 
      meshtype: 

    Returns:

    """
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
    """

    Args:
      filepath: 

    Returns:

    """
    # DOCU add some docstring
    bpy.ops.import_scene.obj(filepath=filepath)


def importStl(filepath):
    """

    Args:
      filepath: 

    Returns:

    """
    # DOCU add some docstring
    bpy.ops.import_mesh.stl(filepath=filepath)


def importDae(filepath):
    """

    Args:
      filepath: 

    Returns:

    """
    # DOCU add some docstring
    bpy.ops.wm.collada_import(filepath=filepath)


def exportObj(obj, path):
    """This function exports a specific object to a chosen path as an .obj

    Args:
      path(str): The path you want the object export to. *without the filename!*
      obj(types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'obj')


def exportStl(obj, path):
    """This function exports a specific object to a chosen path as a .stl

    Args:
      path(str): The path you want the object exported to. *without filename!*
      obj(bpy.types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'stl')


def exportDae(obj, path):
    """This function exports a specific object to a chosen path as a .dae

    Args:
      path(str): The path you want the object exported to. *without filename!*
      obj(bpy.types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'dae')


# registering mesh types with Phobos
mesh_type_dict = {
    'obj': {'export': exportObj, 'import': importObj, 'extensions': ('obj',)},
    'stl': {'export': exportStl, 'import': importStl, 'extensions': ('stl',)},
    'dae': {'export': exportDae, 'import': importDae, 'extensions': ('dae',)},
}
