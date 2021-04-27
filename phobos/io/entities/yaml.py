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
import os
from datetime import datetime
import phobos.defs as defs
from phobos.phoboslog import log


def exportYAML(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
      model(dict): Phobos robot model dictionary
      path(str): filepath to export the robot to (*WITH* filename)

    Returns:

    """
    log("phobos YAML export: Writing model data to " + path, "INFO")
    with open(os.path.join(path, model['name'] + '.yaml'), 'w') as outputfile:
        outputfile.write(
            "# YAML dump of robot model '{}', {}\n".format(
                model['name'], datetime.now().strftime("%Y%m%d_%H:%M")
            )
        )
        outputfile.write(
            "# created with Phobos {} - https://github.com/dfki-ric/phobos\n\n".format(defs.version)
        )

        # write the yaml dump to the file
        outputfile.write(json.dumps(model))


# registering export functions of types with Phobos
entity_type_dict = {'yaml': {'export': exportYAML, 'extensions': ('yaml', 'yml')}}
