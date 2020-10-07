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
import phobos.defs as defs
from phobos.utils.validation import validate
from phobos.phoboslog import log


@validate('material')
def createMaterial(material, logging=False, adjust=False, errors=[]):
    """

    Args:
      material: 
      logging: (Default value = False)
      adjust: (Default value = False)
      errors: (Default value = [])

    Returns:

    """
    # name, diffuse, specular, alpha, diffuse_intensity=1.0, texture=None):
    """Returns a Blender material specified by the material dictionary.

    This creates the appropriate Blender material based on the different dictionary parameters.

    A material needs to contain these keys:
        *name*: unique name of the material
        *diffuse*: [r, g, b, a] color values as floats in range of [0, 1]

    Optional are these keys:
        *specular*: [r, g, b, a] specular color
        *diffuse_intensity*: amount of diffuse reflection (float in range [0, 1])

    Furthermore any generic properties, prepended by a `$` will be added as custom properties to the
    Blender material. E.g. $test/etc would be put to test/etc in the Blender material.
    However, these properties are extracted only in the first layer of hierarchy.

    Args:
        material (dict): representation of a material

    Returns:
        bpy.types.Material
    """
    log("  Creating material {}.".format(material), 'DEBUG')

    mat = bpy.data.materials.new(material['name'])
    mat.diffuse_color = tuple(material['diffuse'])
    #mat.diffuse_shader = 'LAMBERT'

    #if 'diffuse_intensity' in material:
    #    mat.diffuse_intensity = material['diffuse_intensity']
    if 'specular' in material:
        mat.specular_color = tuple(material['specular'][:3])
       # mat.specular_shader = 'COOKTORR'
        mat.specular_intensity = 0.5
    #mat.alpha = material['diffuse'][-1]
    #if mat.alpha < 1.0:
    #    mat.use_transparency = True
    #mat.ambient = 1
    # TODO: implement textures properly
    # if texture is not None:
    #     pass
    #mat.use_fake_user = True

    # write generic custom properties
    for prop in material:
        if prop.startswith('$'):
            for tag in material[prop]:
                mat[prop[1:] + '/' + tag] = material[prop][tag]

    return mat


def createPhobosMaterials():
    """Creates a list of standard materials used in Phobos."""
    materials = bpy.data.materials.keys()
    for materialname in defs.definitions['materials']:
        mat = defs.definitions['materials'][materialname]
        mat['name'] = materialname
        if materialname not in materials:
            createMaterial(mat, logging=False, adjust=False)


def assignMaterial(obj, materialname):
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

    # add material slot never twice
    if materialname not in obj.data.materials:
        obj.data.materials.append(bpy.data.materials[materialname])

    #if obj.data.materials[materialname].use_transparency:
    if obj.data.materials[materialname].diffuse_color[3] < 1.0:
        obj.show_transparent = True
