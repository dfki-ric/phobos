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

from phobos.phoboslog import log
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
from phobos.model.models import deriveModelDictionary
from phobos.utils.io import exportModel


def exportSubmechanisms(model, path):
    """This function exports the submechanisms contained in a robot model.

    Args:
      model(dict): The robot model to export
      path(str): The filepath to export the submechanisms data to

    Returns:

    """
    log("Phobos Submechanisms export: Creating submechanisms data at " + path, "INFO")
    for submechanism in model['submechanisms']:
        root = sUtils.getObjectByProperty('submechanism/name', submechanism['contextual_name'])
        linkobjs = [root] + root['submechanism/spanningtree']
        if 'submechanism/freeloader' in root:
            linkobjs += root['submechanism/freeloader']

        objects = [o for link in linkobjs for o in link.children if o.phobostype in
                   ['visual', 'collision', 'inertial']] + linkobjs
        model = deriveModelDictionary(root, root['submechanism/name'], objects)
        jointname = nUtils.getObjectName(root, 'joint')
        if jointname in model['joints']:
            del model['joints'][jointname]
            log('Removed joint which is not part of submodel: ' + jointname, 'DEBUG')
        exportModel(model, path, ['urdf'])


# registering export functions of types with Phobos
entity_type_dict = {'submechanisms': {'export': exportSubmechanisms,
                                      'extensions': ('urdf',)
                                      }
                    }
