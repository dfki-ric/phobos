'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtinertia.py

Created on 13 Feb 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.

NOTE: If you edit this script, please make sure not to use any imports
not supported by Blender's standard Python distribution. This is a script
intended to be usable on its own and thus should not use external dependencies,
especially none of the other modules of the MARStools package.
'''
import bpy
import marstools.mtdefs as mtdefs
import marstools.mtutility as mtutility

def register():
    print("Registering mtinertia...")

def unregister():
    print("Unregistering mtinertia...")

def getDummyInertia():
    return calculateBoxInertia(0.001, (0.01, 0.01, 0.01,))

def createInertial(link):
    size = (0.05, 0.05, 0.05)
    rotation = (0, 0, 0)
    center = (0, 0, 0)
    inertial = mtutility.createPrimitive('inertial_' + link.name, 'box', size,
                                   mtdefs.layerTypes["inertial"], 'inertial', center, rotation)
    inertial.MARStype = 'inertial'
    bpy.ops.object.select_all(action="DESELECT")
    inertial.select = True
    bpy.context.scene.objects.active = inertial
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    link.select = True
    bpy.context.scene.objects.active = link
    inertial.matrix_world = link.matrix_world
    bpy.ops.object.parent_set()
    return inertial

def calculateInertia(mass, geometry):
    inertia = None
    gt = geometry['geometryType']
    if gt == 'box':
        inertia = calculateBoxInertia(mass, geometry['size'])
    elif gt == 'cylinder':
        inertia = calculateCylinderInertia(mass, geometry["radius"], geometry["height"])
    elif gt == 'sphere':
        inertia = calculateSphereInertia(mass, geometry['radius'])
    elif gt == 'mesh':
        inertia = calculateEllipsoidInertia(mass, geometry['size'])
    return inertia

def calculateBoxInertia(mass, size):
    '''Returns upper diagonal of inertia tensor of a box as tuple.'''
    i = mass / 12
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return [ixx, ixy, ixz, iyy, iyz, izz]

def calculateCylinderInertia(mass, r, h):
    '''Returns upper diagonal of inertia tensor of a cylinder as tuple.'''
    i = mass / 12 * (3 * r**2 + h**2)
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = 0.5 * mass * r**2
    return [ixx, ixy, ixz, iyy, iyz, izz]

def calculateSphereInertia(mass, r):
    '''Returns upper diagonal of inertia tensor of a sphere as tuple.'''
    i = 0.4 * mass * r**2
    ixx = i
    ixy = 0
    ixz = 0
    iyy = i
    iyz = 0
    izz = i
    return [ixx, ixy, ixz, iyy, iyz, izz]

def calculateEllipsoidInertia(mass, size):
    '''Returns upper diagonal of inertia tensor of an ellisoid as tuple.'''
    i = mass / 5
    ixx = i*(size[1]**2 + size[2]**2)
    ixy = 0
    ixz = 0
    iyy = i*(size[0]**2 + size[2]**2)
    iyz = 0
    izz = i*(size[0]**2 + size[1]**2)
    return [ixx, ixy, ixz, iyy, iyz, izz]
