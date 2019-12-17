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
Contains the functions for the light entity.
"""

import phobos.model.models as models
import phobos.utils.selection as sUtils
from phobos.phoboslog import log


def deriveEntity(light, outpath):
    """This function handles a light entity in a scene to export it

    Args:
      entity(bpy.types.Object): The lights root object.
      outpath(str): The path to export to. Not used for light entity
      savetosubfolder(bool): If True data will be exported into subfolders. Not used for light entity
      light: 

    Returns:
      : dict - An entry for the scenes entitiesList

    """
    log("Exporting " + light["entity/name"] + " as a light entity", "INFO")
    entitypose = models.deriveObjectPose(light)
    lightobj = sUtils.getImmediateChildren(light)[0]
    color = lightobj.data.color
    entity = {
        "name": light["entity/name"],
        "type": "light",
        "light_type": "spotlight" if lightobj.data.type == "SPOT" else "omnilight",
        "anchor": light["anchor"] if "anchor" in light else "none",
        "color": {
            "diffuse": [color.r, color.g, color.b],
            "use_specular": lightobj.data.use_specular,  # only specular information currently available
        },
        "position": entitypose["translation"],
        "rotation": entitypose["rotation_quaternion"],
    }
    if entity["light_type"] == "spotlight":
        entity["angle"] = lightobj.data.spot_size
    return entity


#  registering import/export functions of types with Phobos
entity_type_dict = {'light': {'derive': deriveEntity}}
