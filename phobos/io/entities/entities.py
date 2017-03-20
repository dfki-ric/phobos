#!/usr/bin/python
# coding=utf-8

"""
Copyright 2014-2016, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File phobosgui.py

Created on 3 Nov 2016

@author: Kai von Szadkowski
"""

import bpy
import phobos.model.models as models
import phobos.utils.naming as nUtils
from phobos.phoboslog import log

def deriveGenericEntity(entityobj, outpath=None):
    """This function handles an entity of unknown type by simply exporting its custom properties.

    :param entityobj: The object representing the entity.
    :type entityobj: bpy.types.Object
    :param outpath: If True data will be exported into subfolders.
    :type outpath: str
    :return: dict - An entry for the scenes entitiesList

    """
    log("Exporting " + nUtils.getObjectName(entityobj, 'entity') + " as entity of type 'generic", "INFO")
    entity = models.initObjectProperties(entityobj, 'entity', ['geometry'])
    return entity

    # write urdf
    urdf_path = "../urdf/" if structured else ''
    urdf_filename = model['name'] + ".urdf"
    exportModelToURDF(model, os.path.join(path, urdf_path, urdf_filename),
                      '../meshes/' if structured else '')


def exportGenericEntity():
    pass

# registering import/export functions of types with Phobos
entity_type_dict = {'smurf': {'export': exportGenericEntity,
                              'extensions': ('yaml',)}
                    }
