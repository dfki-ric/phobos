#!/usr/bin/python
# coding=utf-8

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

File controllers.py

Created on 24.08.2018

@author: Simon V. Reichel
"""


import bpy
import mathutils
from phobos import defs
from phobos.phoboslog import log
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils


def deriveController(obj):
    return {}


def createController(controller, reference, origin=mathutils.Matrix()):
    """This function creates a new controller specified by its parameters.

    Args:
        controller (dict): phobos representation of the new controller
        reference (bpy_types.Object): object to add a parent relationship to
        origin (mathutils.Matrix, optional): new controllers origin

    Returns:
        bpy.types.Object -- new created controller object
    """
    layers = defs.layerTypes['controller']
    bUtils.toggleLayer(layers, value=True)

