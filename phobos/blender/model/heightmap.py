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
Contains the functions of the heightmap entity.
"""

import os
import shutil

import bpy
from ..utils import io as ioUtils
from ..utils import selection as sUtils
from ..io import blender2phobos
from ..phoboslog import log
from ..utils.io import securepath

# information for structure export
structure_subfolder = "heightmaps"


def deriveHeightmap(entity, outpath):
    """This function handles a heightmap entity in a scene to export it

    Args:
      smurf(bpy.types.Object): The heightmap root object.
      outpath(str): The path to export to.
      savetosubfolder(bool): If True data will be exported into subfolders.
      entity: 

    Returns:
      : dict - An entry for the scenes entitiesList

    """

    heightmap = entity

    # determine outpath for the heightmap export
    heightmap_outpath = securepath(os.path.join(outpath, structure_subfolder))

    log("Exporting " + heightmap["entity/name"] + " as a heightmap entity", "INFO")
    entitypose = blender2phobos.deriveObjectPose(heightmap)
    heightmapMesh = sUtils.getImmediateChildren(heightmap)[0]
    if bpy.data.window_managers[0].heightmapMesh:
        exMesh = heightmapMesh.to_mesh(bpy.context.scene, True, "PREVIEW")
        exMesh.name = "hm_" + heightmap["entity/name"]
        oldMesh = heightmapMesh.data
        heightmapMesh.data = exMesh
        heightmapMesh.modifiers["displace_heightmap"].show_render = False
        heightmapMesh.modifiers["displace_heightmap"].show_viewport = False
        # CHECK are the heightmaps exported to the right directory?
        if bpy.data.window_managers[0].useObj:
            ioUtils.exportObj(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".obj")
        elif bpy.data.window_managers[0].useStl:
            ioUtils.exportStl(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".stl")
        elif bpy.data.window_managers[0].useDae:
            ioUtils.exportDae(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".dae")
        else:
            log("No mesh export type checked! Aborting heightmap export.", "ERROR")
            return {}
        heightmapMesh.modifiers["displace_heightmap"].show_render = True
        heightmapMesh.modifiers["displace_heightmap"].show_viewport = True
        heightmapMesh.data = oldMesh
        bpy.data.meshes.remove(exMesh)
        entry = {
            "name": heightmap["entity/name"],
            "type": "mesh",
            "file": filename,
            "anchor": heightmap["anchor"] if "anchor" in heightmap else "none",
            "position": entitypose["translation"],
            "rotation": entitypose["rotation_quaternion"],
        }

    else:
        imagepath = os.path.abspath(
            os.path.join(os.path.split(bpy.data.filepath)[0], heightmap["image"])
        )
        shutil.copy2(imagepath, heightmap_outpath)
        entry = {
            "name": heightmap["entity/name"],
            "type": "heightmap",
            "file": os.path.join("heightmaps", os.path.basename(imagepath)),
            "anchor": heightmap["anchor"] if "anchor" in heightmap else "none",
            "width": heightmapMesh.dimensions[1],
            "length": heightmapMesh.dimensions[0],
            "height": heightmapMesh.modifiers["displace_heightmap"].strength,
            "position": entitypose["translation"],
            "rotation": entitypose["rotation_quaternion"],
        }
    return entry


# information for the registration in the exporter
entity_type_name = 'heightmap'
