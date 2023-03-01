#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import bpy
import mathutils

from .. import defs
from ..model import models as models
from ..phoboslog import log
from ..utils import blender as bUtils
from ..utils import editing as eUtils
from ..utils import io as ioUtils
from ..utils import naming as nUtils
from ..utils import selection as sUtils


def deriveController(obj):
    """

    Args:
      obj: 

    Returns:

    """
    props = models.initObjectProperties(obj, phobostype='controller')

    # return None if no controller is found (there will always be at least a name in the props)
    if len(props) < 2:
        return None

    if not obj.parent or obj.parent.phobostype not in defs.controllabletypes:
        log(
            (
                "Can not derive controller from {}. "
                + "Insufficient requirements from parent object!"
            ).format(obj.name),
            'ERROR',
        )
        return None

    props['target'] = nUtils.getObjectName(obj.parent)
    log(
        "  Derived controller '{}' for target '{}'.".format(props['name'], props['target']), 'DEBUG'
    )

    return props


def createController(controller, reference, origin=mathutils.Matrix(), annotations=None):
    """This function creates a new controller specified by its parameters.
    
    If an annotation category or the keyword 'all' is specified, the respective annotations for the
    controller will be added as objects.

    Args:
      controller(dict): phobos representation of the new controller
      reference(bpy_types.Object): object to add a parent relationship to
      origin(mathutils.Matrix, optional): new controllers origin (Default value = mathutils.Matrix())
      annotations(list(str, optional): list of annotation keys or 'all' to add to as annotation
    objects (Default value = None)

    Returns:
      : bpy.types.Object -- new created controller object

    """
    bUtils.toggleLayer('controller', value=True)

    # create controller object
    if controller['shape'].startswith('resource'):
        newcontroller = bUtils.createPrimitive(
            controller['name'],
            'box',
            [1, 1, 1],
            [],
            plocation=origin.to_translation(),
            protation=origin.to_euler(),
            pmaterial=controller['material'],
            phobostype='controller',
        )
        # use resource name provided as: "resource:whatever_name"
        resource_obj = ioUtils.getResource(
            ['controller'] + controller['shape'].split('://')[1].split('_')
        )
        if resource_obj:
            log("Assigned resource mesh and materials to new controller object.", 'DEBUG')
            newcontroller.data = resource_obj.data
            newcontroller.scale = (controller['size'],) * 3
        else:
            log("Could not use resource mesh for controller. Default cube used instead.", 'WARNING')
    else:
        newcontroller = bUtils.createPrimitive(
            controller['name'],
            controller['shape'],
            controller['size'],
            layers,
            plocation=origin.to_translation(),
            protation=origin.to_euler(),
            pmaterial=controller['material'],
            phobostype='controller',
        )

    newcontroller.name = controller['name']
    newcontroller['controller/type'] = controller['type']

    # write the custom properties to the controller
    eUtils.addAnnotation(newcontroller, controller['props'], namespace='controller')

    if controller['annotations'] and annotations:
        if annotations == 'all':
            keys = controller['annotations'].keys()
        elif isinstance(annotations, list):
            keys = [key for key in annotations if key in controller['annotations']]
        else:
            keys = []
        for key in keys:
            eUtils.addAnnotationObject(
                newcontroller, controller['annotations'][key], namespace='controller/' + key
            )

    # assign the parent if available
    if reference is not None:
        sUtils.selectObjects([newcontroller, reference], clear=True, active=1)

        if reference.phobostype == 'link':
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
        else:
            bpy.ops.object.parent_set(type='OBJECT')

    return newcontroller
