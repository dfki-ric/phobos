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

File testing.py

Created on 28 Feb 2014

@author: Kai von Szadkowski
"""

import bpy
import mathutils
import phobos.utils.naming as namingUtils

# This is module is a test module to test and understand the data structure of Blender's transforms and to check whether the original
# export code is valid or redundant. One of the points to check is whether or not the "apply transform" option does make a whole lot of sense
# and how it influences the geometry of the bounding boxes.

# Test Scenario 1:
# ----------------
# There are three objects:
# - 1: the overall parent, rotated first by 20 degrees, then child 2 was added, then rotation was increased to 40
# - 2: the first degree child, added as child to 1 when it was rotated 20, rotated 20 itself
# - 3: added to 2 AFTER rotation of 2, thus a different case than 1-2
#
# Both 2 and 3 were moved by -3 (-6 in total for 3) in y-direction before any other transforms were applied.
# For none of the objects, rotations were applied.
#
#
# Test Scenario 2:
# The same as above, only that all rotations were applied to the objects.
#
# results can be found in work/mars/blendertools/test


sep = "------------------------------------------------------"

def calcCenter(bound_box):
    """Returns a mathutils.Vector for the bounding box's center point

    :param bound_box: The bounding box to return the mathutil.Vector for.
    :return: The calculated Vector for the bounding box.

    """
    c = sum((mathutils.Vector(b) for b in bound_box), mathutils.Vector())
    return c / 8

def writeNode(obj):
    """TODO: PLEASE ADD DOC AS FAST AS POSSIBLE!

    :param obj:
    :return:
    """
    size = obj.dimensions

    #get pivot
    obj_pivot = calcCenter(obj.bound_box)

    if obj.parent:
        parent = obj.parent
        parent_pivot = calcCenter(parent.bound_box)
        parentPos = parent.matrix_world * parent_pivot
        childPos = obj.matrix_world * obj_pivot
        childPos = childPos - parentPos
        child_center = parent.matrix_world.to_quaternion().inverted() * childPos        #output: position
        parent_rot = parent.matrix_world.to_quaternion()
        child_rot = obj.matrix_local.to_quaternion()    #output: rotation

def printMatrices(obj):
    """This function prints a blender objects matrices to the console.

    :param obj: The object to print the matrices for.
    :type obj: bpy_types.Object

    """
    print("Transformation Matrices for object:", namingUtils.getObjectName(obj))
    print("World", sep)
    print(obj.matrix_world)
    print("Local", sep)
    print(obj.matrix_local)
    print("Parent Inverse", sep)
    print(obj.matrix_parent_inverse)
    print("Basis", sep)
    print(obj.matrix_basis)
    print("\n\n")

def printRotLoc(obj):
    """This function prints an objects rotation and location to the console.

    :param obj: The object to print the location and rotation for.
    :type obj: bpy_types.Object

    """
    print("Location/Rotation for object:", namingUtils.getObjectName(obj))
    print("rotation_euler", sep)
    print(obj.rotation_euler)
    print("rotation_quaternion", sep)
    print(obj.rotation_quaternion)
    print("Location", sep)
    print(obj.location)
    print("\n\n")

def printBoundBox(obj):
    """This function prints an objects bounding box to the console.

    :param obj: The object to print the bounding box for.
    :type obj: bpy_types.Object

    """
    print("Bounding box for object:", namingUtils.getObjectName(obj))
    for vector in obj.bound_box:
        for value in vector:
            print(value, end="")
        print()

def main():
    """This main function prints the matrices, location, rotation and bounding boxes for all selected objects to the console.

    """
    for obj in bpy.context.selected_objects:
        printMatrices(obj)
        printRotLoc(obj)
        printBoundBox(obj)
        writeNode(obj)

if __name__ == '__main__':
    main()


