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
