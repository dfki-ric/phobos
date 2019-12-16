#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import bpy


def compareOrientation(obj, step):
    """

    Args:
      obj: 
      step: 

    Returns:

    """
    before = sum(obj.dimensions)
    bpy.ops.transform.rotate(value=step, axis=(0, 0, 1))
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    after = sum(obj.dimensions)
    return before, after


def minimizeMeshDimensions(obj, direction, step, epsilon):
    """

    Args:
      obj: 
      direction: 
      step: 
      epsilon: 

    Returns:

    """
    stepsum = 0
    while True:
        before, after = compareOrientation(obj, direction * step)
        if before < after:
            # bpy.ops.transform.rotate(value=-1.0*direction*step, axis=(0, 0, 1))
            # bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
            break
        else:
            stepsum += direction * step
    step = step / 2
    if step > epsilon:
        print(stepsum)
        stepsum += minimizeMeshDimensions(obj, -direction, step, epsilon)
    return stepsum


print('meeerp')
correction = minimizeMeshDimensions(bpy.context.active_object, 1.0, 0.3, 0.00000001)
bpy.ops.transform.rotate(value=-correction, axis=(0, 0, 1))
