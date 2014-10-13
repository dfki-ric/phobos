#!/usr/bin/python

"""
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

@author: Kai von Szadkowski
"""

import bpy
from . import defs

def makeMaterial(name, diffuse, specular, alpha, diffuse_intensity=1.0, texture=None):
    """Returns a Blender material specified by the input parameters"""
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
    return mat

def createPhobosMaterials():
    """Uses makeMaterial() to create a list of standard materials used in Phobos"""
    materials = bpy.data.materials.keys()
    for material in defs.defaultmaterials:
        mat = defs.defaultmaterials[material]
        if not material in materials:
            makeMaterial(material, mat['diffuse'], mat['specular'], mat['alpha'], mat['diffuse_intensity'])
