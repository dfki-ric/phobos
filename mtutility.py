'''
Created on 9 Jan 2014

@author: kavonszadkowski
'''

import bpy
import marstools.mtmaterials as mtmaterials
import marstools.mtdefs as mtdefs

def createPrimitive(pname, ptype, psize, plocation, player, pmaterial):
    try:
        n_layer = int(player)
    except ValueError:
        n_layer = mtdefs.layerTypes[player]
    players = defLayers([n_layer])
    bpy.ops.mesh.primitive_uv_sphere_add(size = psize,
                                  layers = players,
                                  location = plocation)
    obj = bpy.context.object
    obj.name = pname
    obj.data.materials.append(bpy.data.materials[pmaterial])


def defLayers(layerlist):
    layers = 20*[False]
    for layer in layerlist:
        layers[layer+1] = True
    return layers