#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.export.primitive
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadkowski

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

File primitive.py

Created on 12 Sep 2016
"""

import phobos.model.models as models
import phobos.utils.naming as nUtils
import phobos.utils.general as gUtils
from phobos.phoboslog import log
from phobos.model.geometries import deriveGeometry
from phobos.model.poses import deriveObjectPose

def deriveEntity(entity, outpath, savetosubfolder):
    """This function handles a primitive entity in a scene to export it

    :param smurf: The heightmap root object.
    :type smurf: bpy.types.Object
    :param outpath: The path to export to. Not used for primitives
    :type outpath: str
    :param savetosubfolder: If True data will be exported into subfolders. Not used for primitives
    :type savetosubfolder: bool
    :return: dict - An entry for the scenes entitiesList

    """

    primitive = entity

    log("Exporting " + nUtils.getObjectName(primitive, 'entity') + " as entity of type 'primitive", "INFO")
    entity = models.initObjectProperties(primitive, 'entity', ['geometry'])
    pose = deriveObjectPose(primitive)
    entity['geometry'] = deriveGeometry(primitive)
    entity['position'] = {'x': pose['translation'][0],
                          'y': pose['translation'][1],
                          'z': pose['translation'][2]}
    entity['rotation'] = {'w': pose['rotation_quaternion'][0],
                          'x': pose['rotation_quaternion'][1],
                          'y': pose['rotation_quaternion'][2],
                          'z': pose['rotation_quaternion'][3]}
    entity['extend'] = {'x': entity['geometry']['size'][0],
                        'y': entity['geometry']['size'][1],
                        'z': entity['geometry']['size'][2]}
    return entity


def exportPrimitive():
    pass

# registering export functions of types with Phobos
entity_type_dict = {'primitive': {'export': exportPrimitive,
                                  'extensions': ('smurf',)}
                    }
