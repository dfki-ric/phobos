#!/usr/bin/python
# coding=utf-8

from phobos.phoboslog import log
import phobos.utils.selection as sUtils
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
        root = sUtils.getObjectsByProperty('submechanism/name', submechanism['contextual_name'])
        linkobjs = [root] + root['submechanism/freeloader'] + root['submechanism/spanningtree']
        objects = [o for link in linkobjs for o in link.children if o.phobostype in
                   ['visual', 'collision', 'inertial']] + linkobjs
        model = deriveModelDictionary(root, root['submechanism/name'], objects)
        exportModel(model, path, ['urdf'])


# registering export functions of types with Phobos
entity_type_dict = {'submechanisms': {'export': exportSubmechanisms,
                                      'extensions': ('urdf',)
                                      }
                    }
