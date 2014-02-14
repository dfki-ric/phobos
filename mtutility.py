'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtutility.py

Created on 9 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
import marstools.mtmaterials as mtmaterials
import marstools.mtdefs as mtdefs

def register():
    print("Registering mtutility...")


def unregister():
    print("Unregistering mtutility...")

def createPrimitive(pname, ptype, psize, player, pmaterial, plocation, protation = (0, 0, 0)):
    """Generates the primitive specified by the input parameters"""
    try:
        n_layer = int(player)
    except ValueError:
        n_layer = mtdefs.layerTypes[player]
    players = defLayers([n_layer])
    if ptype == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(size = psize[0], layers = players, location = plocation, rotation = protation)
    elif ptype == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=psize[0], depth=psize[1], location = plocation, rotation = protation)
    elif ptype == "cone":
        bpy.ops.mesh.primitive_cone_add(vertices=32,radius=psize[0],depth=psize[1], cap_end=True, location=plocation, rotation = protation)

    obj = bpy.context.object
    obj.name = pname
    obj.data.materials.append(bpy.data.materials[pmaterial])


def defLayers(layerlist):
    """Returns a list of 20 elements encoding the visible layers according to layerlist"""
    layers = 20*[False]
    for layer in layerlist:
        layers[layer+1] = True
    return layers

def returnObjectList(marstype):
    """Returns list of all objects in the current scene matching marstype"""
    objlist = []
    for obj in bpy.context.scene.objects:
        if obj.MARStype == marstype:
            objlist.append(obj)
    return objlist

def getRoot(obj):
    """Finds the root object of a model given one containing object."""
    child = obj
    parent = obj.parent
    while obj.parent != None:
        child = parent
        parent = child.parent
    return child
