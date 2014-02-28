'''
Created on 28 Feb 2014

@author: kavonszadkowski
'''
import bpy
import mathutils
import struct

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


sep = "------------------------------------------------------"

def calcCenter(boundingbox):
    c = [0,0,0]
    for v in boundingbox:
        for i in range(3):
            c[i] += v[i]
    for i in range(3):
        c[i] /= 8.
    return c

def writeNode(obj):
    # get bounding box:
    bBox = obj.bound_box
    center = calcCenter(bBox)
    size = [0.0, 0.0, 0.0]
    size[0] = abs(2.0*(bBox[0][0] - center[0]))
    size[1] = abs(2.0*(bBox[0][1] - center[1]))
    size[2] = abs(2.0*(bBox[0][2] - center[2]))

    #get pivot
    pivot = center
    center = obj.location.copy()
    center += obj.matrix_world.to_quaternion() * mathutils.Vector((pivot[0], pivot[1], pivot[2]))


    obj.rotation_mode = 'QUATERNION'
    q = obj.rotation_quaternion

    if obj.parent:
        parent = obj.parent
        parentIQ = parent.matrix_world.to_quaternion().inverted()
        bBox = parent.bound_box
        pivot2 = calcCenter(bBox)
        v = mathutils.Vector((pivot2[0], pivot2[1], pivot2[2]))
        #v = parent.matrix_world.to_quaternion() * v
        parentPos = parent.matrix_world * v
        childPos = obj.matrix_world * mathutils.Vector((pivot[0], pivot[1], pivot[2]))
        childPos = childPos - parentPos
        center = parentIQ * childPos
        parentRot = parent.matrix_world.to_quaternion()
        childRot = obj.matrix_world.to_quaternion()
        childRot = parentRot.rotation_difference(childRot)
        q = childRot

def printMatrices(obj):
    print("Transformation Matrices for object:", obj.name)
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
    print("Location/Rotation for object:", obj.name)
    print("rotation_euler", sep)
    print(obj.rotation_euler)
    print("rotation_quaternion", sep)
    print(obj.rotation_quaternion)
    print("Location", sep)
    print(obj.location)
    print("\n\n")

def printBoundBox(obj):


def main():
    for obj in bpy.context.selected_objects:
        printMatrices(obj)
        printRotLoc(obj)
        printBoundBox(obj)

if __name__ == '__main__':
    main()


