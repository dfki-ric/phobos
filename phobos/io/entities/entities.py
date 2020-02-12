#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
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
