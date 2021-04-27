#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import json
from datetime import datetime
from phobos.defs import version
from phobos.defs import repository
from phobos.utils import io as ioUtils
from phobos.utils.general import roundFloatsInDict
from phobos.phoboslog import log


def exportSMURFScene(entities, path):
    """Exports an arranged scene into SMURFS. It will export only entities
    with a valid entity/name, and entity/type property.

    Args:
      selected_only(bool): If True only selected entities get exported.
      subfolder(bool): If True the models are exported into separate subfolders
      entities: 
      path: 

    Returns:

    """
    log("Exporting scene to " + path + '.smurfs', "INFO")
    with open(path + '.smurfs', 'w') as outputfile:
        sceneinfo = (
            "# SMURF scene created at "
            + path
            + " "
            + datetime.now().strftime("%Y%m%d_%H:%M")
            + "\n"
        )
        log(sceneinfo, "INFO")
        sceneinfo += "# created with Phobos " + version + " - " + repository + "\n\n"
        ioUtils.securepath(path)
        outputfile.write(sceneinfo)
        entitiesdict = roundFloatsInDict(
            {'entities': entities}, ioUtils.getExpSettings().decimalPlaces
        )
        outputfile.write(json.dumps(entitiesdict, indent=2))


# registering import/export functions of types with Phobos
scene_type_dict = {'sdf world': {'export': exportSMURFScene, 'extensions': ('smurfs',)}}
