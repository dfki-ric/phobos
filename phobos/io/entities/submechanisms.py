#!/usr/bin/python
# coding=utf-8

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
