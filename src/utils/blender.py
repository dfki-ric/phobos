#!/usr/bin/python

"""
.. module:: phobos.utils.blender
    :platform: Unix, Windows, Mac
    :synopsis: This module contains functions o manipulate blender objects and interact with blender functionalities

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert

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
"""

import phobos.utils.selection as selection
import phobos.defs as defs
import phobos.materials as materials
import bpy
from phobos.logging import log

import yaml


def printMatrices(obj, info=None):
    if not info:
        info = selection.getObjectName(obj)
    print("\n----------------", info, "---------------------\n",
          "local:\n", obj.matrix_local,
          "\n\nworld:\n", obj.matrix_world,
          "\n\nparent_inverse:\n", obj.matrix_parent_inverse,
          "\n\nbasis:\n", obj.matrix_basis)


def assignMaterial(obj, materialname):
    if materialname not in bpy.data.materials:
        if materialname in defs.defaultmaterials:
            materials.createPhobosMaterials()
        else:
            # print("###ERROR: material to be assigned does not exist.")
            log("Material to be assigned does not exist.", "ERROR")
            return None
    obj.data.materials.append(bpy.data.materials[materialname])
    if bpy.data.materials[materialname].use_transparency:
        obj.show_transparent = True


def createPrimitive(pname, ptype, psize, player=0, pmaterial="None", plocation=(0, 0, 0), protation=(0, 0, 0),
                    verbose=False):
    """Generates the primitive specified by the input parameters"""
    if verbose:
        log(ptype + psize, "INFO", "createPrimitive")
    try:
        # n_layer = bpy.context.scene.active_layer
        n_layer = int(player)
    except ValueError:
        n_layer = defs.layerTypes[player]
    players = defLayers([n_layer])
    bpy.context.scene.layers[n_layer] = True  # the layer has to be active to prevent problems with object placement
    if ptype == "box":
        bpy.ops.mesh.primitive_cube_add(layers=players, location=plocation, rotation=protation)
        obj = bpy.context.object
        obj.dimensions = psize
    if ptype == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(size=psize, layers=players, location=plocation, rotation=protation)
    elif ptype == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=psize[0], depth=psize[1], layers=players,
                                            location=plocation, rotation=protation)
    elif ptype == "cone":
        bpy.ops.mesh.primitive_cone_add(vertices=32, radius=psize[0], depth=psize[1], cap_end=True, layers=players,
                                        location=plocation, rotation=protation)
    elif ptype == 'disc':
        bpy.ops.mesh.primitive_circle_add(vertices=psize[1], radius=psize[0], fill_type='TRIFAN', location=plocation,
                                          rotation=protation, layers=players)
    obj = bpy.context.object
    obj.name = pname
    if pmaterial != 'None':
        assignMaterial(obj, pmaterial)
    return obj


def toggleLayer(index, value=None):
    if value:
        bpy.context.scene.layers[index] = value
    else:
        bpy.context.scene.layers[index] = not bpy.context.scene.layers[index]


def defLayers(layerlist):
    """Returns a list of 20 elements encoding the visible layers according to layerlist"""
    if type(layerlist) is not list:
        layerlist = [layerlist]
    layers = 20 * [False]
    for layer in layerlist:
        layers[layer] = True
    return layers

def updateTextFile(textfilename, newContent):
    try:
        bpy.data.texts.remove(bpy.data.texts[textfilename])
    except KeyError:
        pass #Not important. Just create.
    createNewTextfile(textfilename, newContent)

def readTextFile(textfilename):
    try:
        return "\n".join([l.body for l in bpy.data.texts[textfilename].lines])
    except KeyError:
        log("No text file " + textfilename + " found. Setting", "INFO")
        return ""

def createNewTextfile(textfilename, contents):
    for text in bpy.data.texts:
        text.tag = True
    bpy.ops.text.new()
    newtext = None
    for text in bpy.data.texts:
        if not text.tag:
            newtext = text
    for text in bpy.data.texts:
        text.tag = False
    newtext.name = textfilename
    bpy.data.texts[textfilename].write(contents)


def openScriptInEditor(scriptname):
    if scriptname in bpy.data.texts:
        for area in bpy.context.screen.areas:
            if area.type == 'TEXT_EDITOR':
                area.spaces.active.text = bpy.data.texts[scriptname]
    else:
        log("There is no script named " + scriptname + "!", "ERROR")


def cleanObjectProperties(props):
    """Cleans a predefined list of Blender-specific or other properties from the dictionary."""
    getridof = ['phobostype', '_RNA_UI', 'cycles_visibility', 'startChain', 'endChain', 'masschanged']
    if props:
        for key in getridof:
            if key in props:
                del props[key]
    return props
