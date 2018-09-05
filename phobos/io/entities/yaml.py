# TODO add shebang and intro documentation

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

import yaml
import os
from datetime import datetime
import phobos.defs as defs
from phobos.phoboslog import log


def exportYAML(model, path):
    """This function exports a given robot model to a specified filepath as YAML.

    Args:
        model (dict):  Phobos robot model dictionary
        path (str): filepath to export the robot to (*WITH* filename)
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
        outputfile.write(yaml.dump(model))


# registering export functions of types with Phobos
entity_type_dict = {'yaml': {'export': exportYAML, 'extensions': ('yaml', 'yml')}}
