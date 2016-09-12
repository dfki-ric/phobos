#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Sebastian Klemp

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

File heightmap_entity.py

Created on 13 Feb 2014
"""

import os
import shutil
import bpy
import phobos.robotdictionary as robotdictionary
import phobos.utils.selection as sUtils
import phobos.utils.export as eUtils
from phobos.utils.general import securepath
from phobos.logging import log

def deriveEntity(entity, outpath, savetosubfolder):
    """This function handles a heightmap entity in a scene to export it

    :param smurf: The heightmap root object.
    :type smurf: bpy.types.Object
    :param outpath: The path to export to.
    :type outpath: str
    :param savetosubfolder: If True data will be exported into subfolders.
    :type savetosubfolder: bool
    :return: dict - An entry for the scenes entitiesList

    """

    heightmap = entity

    # determine outpath for the heightmap export
    heightmap_outpath = securepath(os.path.join(outpath, "heightmaps") if savetosubfolder else outpath)


    log("Exporting " + heightmap["entity/name"] + " as a heightmap entity", "INFO")
    entitypose = robotdictionary.deriveObjectPose(heightmap)
    heightmapMesh = sUtils.getImmediateChildren(heightmap)[0]
    if bpy.data.worlds[0].heightmapMesh:
        exMesh = heightmapMesh.to_mesh(bpy.context.scene, True, "PREVIEW")
        exMesh.name = "hm_" + heightmap["entity/name"]
        oldMesh = heightmapMesh.data
        heightmapMesh.data = exMesh
        heightmapMesh.modifiers["displace_heightmap"].show_render = False
        heightmapMesh.modifiers["displace_heightmap"].show_viewport = False
        if bpy.data.worlds[0].useObj:
            eUtils.exportObj(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".obj")
        elif bpy.data.worlds[0].useBobj:
            eUtils.exportBobj(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".bobj")
        elif bpy.data.worlds[0].useStl:
            eUtils.exportStl(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".stl")
        elif bpy.data.worlds[0].useDae:
            eUtils.exportDae(heightmap_outpath, heightmapMesh)
            filename = os.path.join("heightmaps", exMesh.name + ".dae")
        else:
            log("No mesh export type checked! Aborting heightmap export.", "ERROR", __name__+".handleScene_heightmap")
            return {}
        heightmapMesh.modifiers["displace_heightmap"].show_render = True
        heightmapMesh.modifiers["displace_heightmap"].show_viewport = True
        heightmapMesh.data = oldMesh
        bpy.data.meshes.remove(exMesh)
        entry = {"name": heightmap["entity/name"],
                 "type": "mesh",
                 "file": filename,
                 "anchor": heightmap["anchor"] if "anchor" in heightmap else "none",
                 "position": entitypose["translation"],
                 "rotation": entitypose["rotation_quaternion"]
                 }

    else:
        imagepath = os.path.abspath(os.path.join(os.path.split(bpy.data.filepath)[0], heightmap["image"]))
        shutil.copy2(imagepath, heightmap_outpath)
        entry = {"name": heightmap["entity/name"],
                 "type": "heightmap",
                 "file": os.path.join("heightmaps", os.path.basename(imagepath)),
                 "anchor": heightmap["anchor"] if "anchor" in heightmap else "none",
                 "width": heightmapMesh.dimensions[1],
                 "length": heightmapMesh.dimensions[0],
                 "height": heightmapMesh.modifiers["displace_heightmap"].strength,
                 "position": entitypose["translation"],
                 "rotation": entitypose["rotation_quaternion"]
                 }
    return entry

# information for the registration in the exporter
entity_type_name = 'heightmap'