#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains all functions to model links within Blender.
"""

import re

import bpy
import mathutils

from ..phoboslog import log
from ..utils import editing as eUtils
from ..utils import naming as nUtils
from ..utils import blender as bUtils


def getGeometricElements(link):
    """Returns all geometric elements of a link, i.e. 'visual' and 'collision' objects.

    Args:
      link(dict): definition of link
    Returns(list): lists of visual and collision object definitions

    Returns:

    """
    visuals = []
    collisions = []
    if 'visual' in link:
        visuals = [link['visual'][v] for v in link['visual']]
    if 'collision' in link:
        collisions = [link['collision'][v] for v in link['collision']]
    return visuals, collisions


def deriveLinkfromObject(obj, scale=None, parent_link=True, parent_objects=True, reparent_children=True, nameformat=''):
    """Derives a link from an object using its name, transformation and parenting.

    Args:
      obj(bpy_types.Object): object to derive a link from
      scale(float, optional): scale factor for bone size (Default value = 0.2)
      parent_link(bool, optional): whether to automate the parenting of the new link or not. (Default value = True)
      parent_objects(bool, optional): whether to parent all the objects to the new link or not (Default value = False)
      nameformat(str, optional): re-formatting template for obj names (Default value = '')

    Returns:
      : newly created link

    """
    log('Deriving link from ' + nUtils.getObjectName(obj), level="INFO")
    # create armature/bone
    bUtils.toggleLayer('link', True)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.armature_add()
    newlink = bpy.context.active_object
    newlink.name = obj.name + "_link"
    newlink.matrix_world = obj.matrix_world
    newlink.phobostype = 'link'
    bound_box = (
        max([c[0] for c in obj.bound_box]),
        max([c[1] for c in obj.bound_box]),
        max([c[2] for c in obj.bound_box]),
    )
    print(bound_box, type(bound_box), type(bound_box[0]))
    newlink.scale = [max(bound_box)]*3 if scale is None else scale
    if obj.parent is not None and parent_link:
        eUtils.parentObjectsTo(newlink, obj.parent)
    if parent_objects:
        eUtils.parentObjectsTo(obj, newlink)
    if reparent_children:
        eUtils.parentObjectsTo(list(obj.children), newlink)
    return newlink


def setLinkTransformations(model, parent):
    """Assigns the transformations recursively for a model parent link according to the model.
    
    This needs access to the **object** key of a link entry in the specified model.
    The transformations for each link object are extracted from the specified model and applied to
    the Blender object.

    Args:
      parent(dict): parent link you want to set the children for.
      model(dict): model dictionary containing the **object** key for each link

    Returns:

    """
    #todo: bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
    for chi in parent['children']:
        child = model['links'][chi]

        # apply transform as saved in model
        location = mathutils.Matrix.Translation(child['pose']['translation'])
        rotation = (
            mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        )

        log("Transforming link {0}.".format(child['name']), 'DEBUG')
        transform_matrix = location @ rotation
        child['object'].matrix_local = transform_matrix

        # traverse the tree
        setLinkTransformations(model, child)
