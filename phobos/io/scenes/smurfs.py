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

File smurfs.py

Created on 3 Nov 2016

@author: Kai von Szadkowski
"""

import yaml
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
    log("Exporting scene to " + path+'.smurfs', "INFO")
    with open(path + '.smurfs', 'w') as outputfile:
        sceneinfo = "# SMURF scene created at " + path + " " + datetime.now().strftime("%Y%m%d_%H:%M") + "\n"
        log(sceneinfo, "INFO")
        sceneinfo += "# created with Phobos " + version + " - " + repository + "\n\n"
        ioUtils.securepath(path)
        outputfile.write(sceneinfo)
        entitiesdict = roundFloatsInDict({'entities': entities}, ioUtils.getExpSettings().decimalPlaces)
        outputfile.write(yaml.dump(entitiesdict))

# registering import/export functions of types with Phobos
scene_type_dict = {'smurfs': {'export': exportSMURFScene,
                              'extensions': ('smurfs',)}
                   }
