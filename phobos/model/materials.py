#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.materials
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadkowski

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

File materials.py

Created on 7 Jan 2014

"""

import bpy
import phobos.defs as defs
from phobos.phoboslog import log


def createMaterial(name, diffuse, specular, alpha, diffuse_intensity=1.0, texture=None):
    """Returns a Blender material specified by the input parameters

    :param name: The name of the new material.
    :type name: str
    :param diffuse: The color of the new material.
    :type diffuse: float array with 3 elements.
    :param specular: The specular color of the new material.
    :type specular: float array with 3 elements.
    :param alpha: The transparency of the material.
    :type alpha: float in [0,1.0].
    :param diffuse_intensity: The amount of diffuse reflection. The default is 1.0.
    :type diffuse_intensity: float in [0,1.0].
    :param texture: NOT IMPEMENTED YET.
    :type texture: NOT IMPLEMENTED YET.
    :return: bpy.types.Material

    """
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = diffuse
    mat.diffuse_shader = 'LAMBERT'
    mat.diffuse_intensity = diffuse_intensity
    mat.specular_color = specular
    mat.specular_shader = 'COOKTORR'
    mat.specular_intensity = 0.5
    mat.alpha = alpha
    if alpha < 1.0:
        mat.use_transparency = True
    mat.ambient = 1
    if texture is not None:
        # TODO: implement textures properly
        pass
    mat.use_fake_user = True
    return mat


def createPhobosMaterials():
    """Creates a list of standard materials used in Phobos.

    """
    materials = bpy.data.materials.keys()
    for material in defs.definitions['materials']:
        mat = defs.definitions['materials'][material]
        if not material in materials:
            createMaterial(material, mat['diffuse'], mat['specular'], mat['alpha'], mat['diffuse_intensity'])


def assignMaterial(obj, materialname):
    """Assigns a material to an object, avoiding creating multiple copies.

    :param obj: The object to assign the material to.
    :type obj: bpy.types.Object
    :param materialname: The materials name.
    :type materialname: str

    """
    if materialname not in bpy.data.materials:
        if materialname in defs.definitions['materials']:
            createPhobosMaterials()
        else:
            log("Material to be assigned does not exist.", "ERROR", "assignMaterial")
            return None
    obj.data.materials.append(bpy.data.materials[materialname])
    if bpy.data.materials[materialname].use_transparency:
        obj.show_transparent = True
