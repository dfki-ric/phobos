#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.exporter
    :platform: Unix, Windows, Mac
    :synopsis: TODO: INSERT TEXT HERE

.. moduleauthor:: Kai von Szadowski

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

File export.py

Created on 13 Feb 2014
"""

import os
from datetime import datetime
import yaml
import bpy
import phobos.robotdictionary as robotdictionary
import phobos.defs as defs
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.general as gUtils
from phobos.utils.general import securepath, roundVector
from phobos.logging import log
from phobos.export import entity_types


def register():
    """This function is called when this module is registered to blender.

    """
    print("Registering export...")


def unregister():
    """This function is called when this module is unregistered from blender.

    """
    print("Unregistering export...")


def bakeModel(objlist, path, modelname):
    """This function gets a list of objects and creates a single, simplified mesh from it and exports it to .stl.

    :param objlist: The list of blender objects to join and export as simplified stl file.
    :type objlist: list
    :param path: The path to export the stl file to *without filename*
    :type path: str
    :param modelname: The new models name and filename.
    :type modelname: str

    """
    visuals = [o for o in objlist if ("phobostype" in o and o.phobostype == "visual")]
    sUtils.selectObjects(visuals, active=0)
    log("Copying objects for joining...", "INFO")
    print("Copying objects for joining...")
    bpy.ops.object.duplicate(linked=False, mode='TRANSLATION')
    log("joining...", "INFO")
    print("joining...")
    bpy.ops.object.join()
    obj = bpy.context.active_object
    log("Deleting vertices...", "INFO")
    print("Deleting vertices...")
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.select_all(action='TOGGLE')
    bpy.ops.mesh.select_all(action='TOGGLE')
    bpy.ops.mesh.remove_doubles()
    bpy.ops.object.editmode_toggle()
    log("Adding modifier...", "INFO")
    print("Adding modifier...")
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.context.object.modifiers["Decimate"].decimate_type = 'DISSOLVE'
    log("Applying modifier...", "INFO")
    print("Applying modifier...")
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Decimate")
    obj.name = modelname + "_bake.stl"
    bpy.ops.export_mesh.stl(filepath=os.path.join(path, obj.name))
    obj.select = True
    bpy.ops.object.delete()
    log("Done baking...", "INFO")
    print("Done baking...")


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
    entities = [e for e in [obj for obj in bpy.context.scene.objects if sUtils.isEntity(obj)]
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
                entry = entity_types[entity["entity/type"]].deriveEntity(entity,outpath,subfolder)  # known entity export
            else:
                log("Required method ""deriveEntity"" not implemented", "ERROR")
        else:  # generic entity export
            entry = deriveGenericEntity(entity)
        outputlist.append(entry)

    with open(os.path.join(outpath, bpy.data.worlds['World'].sceneName + '.smurfs'),
              'w') as outputfile:
        sceneinfo = "# SMURF scene " + bpy.data.worlds['World'].sceneName + "; created " + datetime.now().strftime("%Y%m%d_%H:%M") + "\n"
        sceneinfo += "# created with Phobos " + defs.version + " - https://github.com/rock-simulation/phobos\n\n"
        outputfile.write(sceneinfo)
        epsilon = 10**(-bpy.data.worlds[0].decimalPlaces)  # TODO: implement this separately
        entitiesdict = gUtils.epsilonToZero({'entities': outputlist}, epsilon, bpy.data.worlds[0].decimalPlaces)
        outputfile.write(yaml.dump(entitiesdict))


def deriveGenericEntity(entityobj, outpath=None):
    """This function handles an entity of unknown type by simply exporting its custom properties.

    :param entityobj: The object representing the entity.
    :type entityobj: bpy.types.Object
    :param outpath: If True data will be exported into subfolders.
    :type outpath: str
    :return: dict - An entry for the scenes entitiesList

    """
    log("Exporting " + nUtils.getObjectName(entityobj, 'entity') + " as entity of type 'generic", "INFO")
    entity = robotdictionary.initObjectProperties(entityobj, 'entity', ['geometry'])
    return entity
