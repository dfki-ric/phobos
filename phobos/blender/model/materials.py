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
Contains the functions required to model a material in Blender.
"""

import bpy

from .. import defs
from ..phoboslog import log


def createPhobosMaterials():
    """Creates a list of standard materials used in Phobos."""
    materials = bpy.data.materials.keys()
    for materialname in defs.definitions['materials']:
        mat = defs.definitions['materials'][materialname]
        mat['name'] = materialname
        if materialname not in materials:
            new_material = bpy.data.materials.new(mat['name'])
            new_material.diffuse_color = tuple(mat['diffuse'])
            if 'specular' in mat:
                new_material.specular_color = tuple(mat['specular'][:3])
                new_material.specular_intensity = 0.5


def assignMaterial(obj, materialname, override=True):
    """Assigns a material by name to an object.
    
    This avoids creating multiple copies and also omits duplicate material slots in the specified
    object.

    Args:
      obj(bpy.types.Object): The object to assign the material to.
      materialname(str): name of the material

    Returns:

    """
    if materialname not in bpy.data.materials:
        if materialname in defs.definitions['materials']:
            createPhobosMaterials()
        else:
            log("Material '" + materialname + "' is not defined.", "ERROR")
            return None

    if override and len(obj.data.materials) >= 1:
        obj.data.materials.clear()

    # add material slot never twice
    if materialname not in obj.data.materials:
        obj.data.materials.append(bpy.data.materials[materialname])

    #if obj.data.materials[materialname].use_transparency:
    if obj.data.materials[materialname].diffuse_color[3] < 1.0:
        obj.show_transparent = True
