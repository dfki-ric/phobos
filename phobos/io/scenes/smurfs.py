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

import os
import yaml
from datetime import datetime
import bpy
from phobos.defs import version
from phobos.utils.general import securepath, epsilonToZero
from phobos.export import entity_types
from phobos.utils.selection import isEntity
from phobos.logging import log
from entities import deriveGenericEntity

def exportSMURFsScene(selected_only=True, subfolder=True):
    """Exports an arranged scene into SMURFS. It will export only entities
    with a valid entity/name, and entity/type property.

    :param selected_only: If True only selected entities get exported.
    :type selected_only: bool
    :param subfolder: If True the models are exported into separate subfolders
    :type subfolder: bool

    """

    outputlist = []

    # identify all entities in the scene
    entities = [e for e in [obj for obj in bpy.context.scene.objects if isEntity(obj)]
                if ((selected_only and e.select) or not selected_only)]
    if len(entities) == 0:
        log("There are no entities to export!", "WARNING", __name__+".exportSMURFsScene")
        return
    # determine outpath for this scene
    if bpy.data.worlds[0].relativePath:
        outpath = securepath(os.path.expanduser(os.path.join(bpy.path.abspath("//"), bpy.data.worlds[0].path)))
    else:
        outpath = securepath(os.path.expanduser(bpy.data.worlds[0].path))
    log("Exporting scene to " + outpath, "INFO", "exportSMURFsScene")
    for entity in entities:
        log("Exporting " + str(entity["entity/name"]) + " to SMURFS", "INFO")
        if entity["entity/type"] in entity_types:
            if hasattr(entity_types[entity["entity/type"]], 'deriveEntity'):
                entry = entity_types[entity["entity/type"]].deriveEntity(entity, outpath, subfolder)  # known entity export
            else:
                log("Required method ""deriveEntity"" not implemented", "ERROR")
        else:  # generic entity export
            entry = deriveGenericEntity(entity)
        outputlist.append(entry)

    with open(os.path.join(outpath, bpy.data.worlds['World'].sceneName + '.smurfs'),
              'w') as outputfile:
        sceneinfo = "# SMURF scene " + bpy.data.worlds['World'].sceneName + "; created " + datetime.now().strftime("%Y%m%d_%H:%M") + "\n"
        sceneinfo += "# created with Phobos " + version + " - https://github.com/rock-simulation/phobos\n\n"
        outputfile.write(sceneinfo)
        epsilon = 10**(-bpy.data.worlds[0].decimalPlaces)  # TODO: implement this separately
        entitiesdict = epsilonToZero({'entities': outputlist}, epsilon, bpy.data.worlds[0].decimalPlaces)
        outputfile.write(yaml.dump(entitiesdict))
