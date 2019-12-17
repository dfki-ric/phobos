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
Contains the functions of the primitive entity.
"""

import phobos.model.models as models
import phobos.utils.naming as nUtils
import phobos.utils.general as gUtils
from phobos.phoboslog import log
from phobos.model.geometries import deriveGeometry
from phobos.model.poses import deriveObjectPose


def deriveEntity(primitive, outpath):
    """This function handles a primitive entity in a scene to export it
    
    # TODO is this even a heightmap?

    Args:
      smurf(bpy.types.Object): The heightmap root object.
      outpath(str): The path to export to. Not used for primitives
      savetosubfolder(bool): If True data will be exported into subfolders. Not used for primitives
      primitive: 

    Returns:
      : dict - An entry for the scenes entitiesList

    """
    entity = models.initObjectProperties(primitive, 'entity', ['geometry'])
    pose = deriveObjectPose(primitive)
    entity['geometry'] = deriveGeometry(primitive)
    entity['position'] = {
        'x': pose['translation'][0],
        'y': pose['translation'][1],
        'z': pose['translation'][2],
    }
    entity['rotation'] = {
        'w': pose['rotation_quaternion'][0],
        'x': pose['rotation_quaternion'][1],
        'y': pose['rotation_quaternion'][2],
        'z': pose['rotation_quaternion'][3],
    }
    if 'radius' in entity['geometry']:
        entity['radius'] = entity['geometry']['radius']
    # entity['extend'] = {'x': entity['geometry']['size'][0],
    #                    'y': entity['geometry']['size'][1],
    #                    'z': entity['geometry']['size'][2]}
    entity['extend'] = {
        'x': primitive.dimensions[0],
        'y': primitive.dimensions[1],
        'z': primitive.dimensions[2],
    }
    return entity


def exportPrimitive():
    """TODO Missing documentation"""
    pass


# registering export functions of types with Phobos
entity_type_dict = {'primitive': {'derive': deriveEntity}}
