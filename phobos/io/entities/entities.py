#!/usr/bin/python
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------------

import bpy
import phobos.model.models as models
import phobos.utils.naming as nUtils
from phobos.phoboslog import log

# TODO this function won't work at all... Finish this
def deriveGenericEntity(entityobj, outpath=None):
    """This function handles an entity of unknown type by simply exporting its custom properties.

    Args:
      entityobj(bpy.types.Object): The object representing the entity.
      outpath(str, optional): If True data will be exported into subfolders. (Default value = None)

    Returns:
      : dict - An entry for the scenes entitiesList

    """
    log(
        "Exporting " + nUtils.getObjectName(entityobj, 'entity') + " as entity of type 'generic",
        "INFO",
    )
    entity = models.initObjectProperties(entityobj, 'entity', ['geometry'])
    return entity

    # write urdf
    urdf_path = "../urdf/" if structured else ''
    urdf_filename = model['name'] + ".urdf"
    exportModelToURDF(
        model, os.path.join(path, urdf_path, urdf_filename), '../meshes/' if structured else ''
    )


def exportGenericEntity(entity, outpath):
    """

    Args:
      entity: 
      outpath: 

    Returns:

    """
    pass


# registering import/export functions of types with Phobos
entity_type_dict = {'generic': {'export': exportGenericEntity, 'extensions': ('yaml',)}}
