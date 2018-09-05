# TODO SHEBANG

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
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
