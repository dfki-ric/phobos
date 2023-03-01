#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------


def getLargestDimension(geometry):
    """

    Args:
      geometry: 

    Returns:

    """
    # DOCU add some docstring
    if geometry['type'] == 'box':
        return max(geometry['size'])
    if geometry['type'] == 'cylinder':
        return max((geometry['radius'], geometry['length']))
    if geometry['type'] == 'sphere':
        return geometry['radius']
    if geometry['type'] == 'mesh':
        # scale would make no sense here for an absolute measure
        return max(geometry['size']) if 'size' in geometry else 0.2


def deriveScale(obj):
    """Returns the scale of the specified object.
    
    Object scale is gathered from the matrix_world, as the link scales in Blender might change the
    mesh scale, too.

    Args:
      obj(bpy.types.Object): object to derive the scale of

    Returns:
      list: three scale floats (x, y, z) combined from all parents and the object itself

    """
    return list(obj.matrix_world.to_scale())

