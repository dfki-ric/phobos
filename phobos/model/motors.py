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
from phobos import defs
from phobos.phoboslog import log
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils


def createMotor(motor, parentobj, origin=mathutils.Matrix(), addcontrollers=False):
    """This function creates a new motor specified by its parameters.

    If *addcontrollers* is set, a controller object will be created from the controller definition
    which is specified in the motor dictionary (key *controller*).

    Args:
      motor(dict): phobos representation of the new motor.
      parentobj(bpy_types.Object): object to parent new motor to
      origin(mathutils.Matrix, optional): new motors origin (Default value = mathutils.Matrix())
      addcontrollers(bool, optional): whether to add the defined controller as object (Default value = False)

    Returns:
      bpy.types.Object: new motor object or a list of the new motor_object and the new controller object

    """
    bUtils.toggleLayer('motor', value=True)

    primitive_name = ''

    # create name if not given by motor dict
    if not 'name' in motor or len(motor['name']) == 0:
        motor['name'] = parentobj.name
        primitive_name = "motor_" + motor['name']
    else:
        primitive_name = motor['name']

    primitive_name = ''

    # create name if not given by motor dict
    if not 'name' in motor or len(motor['name']) == 0:
        motor['name'] = parentobj.name
        primitive_name = "motor_" + motor['name']
    else:
        primitive_name = motor['name']

    # create motor object
    if motor['shape'].startswith('resource'):
        newmotor = bUtils.createPrimitive(
            primitive_name,
            'box',
            [1, 1, 1],
            [],
            plocation=origin.to_translation(),
            protation=origin.to_euler(),
            pmaterial=motor['material'],
            phobostype='motor',
        )
        # use resource name provided as: "resource:whatever_name"
        resource_obj = ioUtils.getResource(['motor'] + motor['shape'].split('://')[1].split('_'))
        if resource_obj:
            log("Assigned resource mesh and materials to new motor object.", 'DEBUG')
            newmotor.data = resource_obj.data
            newmotor.scale = (motor['size'],) * 3
        else:
            log("Could not use resource mesh for motor. Default cube used instead.", 'WARNING')
    else:
        newmotor = bUtils.createPrimitive(
            primitive_name,
            motor['shape'],
            motor['size'],
            [],
            plocation=origin.to_translation(),
            protation=origin.to_euler(),
            pmaterial=motor['material'],
            phobostype='motor',
        )

    # assign the parent if available
    if parentobj is not None:
        sUtils.selectObjects([newmotor, parentobj], clear=True, active=1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')

    # set motor properties
    newmotor.phobostype = 'motor'
    # should not be nessaccary: newmotor.name = motor['name']
    defname = motor['defname']

    # write the custom properties to the motor
    eUtils.addAnnotation(newmotor, motor['props'], namespace='motor', ignore=['defname'])
    # fix motor name since it can differe from object name
    newmotor['motor/name'] = motor['name']

    if 'controller' in defs.definitions['motors'][defname] and addcontrollers:
        import phobos.model.controllers as controllermodel

        motorcontroller = defs.definitions['motors'][defname]['controller']
        controllerdefs = ioUtils.getDictFromYamlDefs(
            'controller', motorcontroller, newmotor.name + '_controller'
        )
        newcontroller = controllermodel.createController(
            controllerdefs, newmotor, origin=newmotor.matrix_world, annotations='all'
        )
    else:
        newcontroller = None

    # select the new motor
    sUtils.selectObjects(
        [newmotor] if not newcontroller else [newmotor, newcontroller], clear=True, active=0
    )
    return newmotor if not newcontroller else [newmotor, newcontroller]


def deriveMotor(obj, jointdict=None):
    """Derives motor information from an object.

    Args:
      obj(bpy_types.Object): Blender object to derive the motor from
      jointdict(dict, optional): phobos representation of the respective joint (Default value = None)

    Returns:
      : dict -- phobos representation of a motor

    """
    import phobos.model.models as models
    import phobos.model.controllers as controllermodel

    props = models.initObjectProperties(obj, phobostype='motor')

    # return None if no motor is attached (there will always be at least a name in the props)
    if len(props) < 2:
        return None

    # make sure the parent is a joint
    if not obj.parent or obj.parent.phobostype != 'link' or 'joint/type' not in obj.parent:
        log(
            "Can not derive motor from {}. Insufficient requirements from parent object!".format(
                obj.name
            ),
            'ERROR',
        )
        return None

    props['joint'] = nUtils.getObjectName(obj.parent, phobostype='joint')

    # todo: transfer joint limits to motor properties
    # check for a mimic motor
    for k in (obj.parent).keys():
        # Check for mimic motor
        if "mimic" in k:
            # Find the name
            mimic_driver = sUtils.getObjectByName((obj.parent)['joint/mimic_joint'], phobostypes = ['link'])
            c_motor = sUtils.getImmediateChildren(mimic_driver, phobostypes = ['motor'])
            props['mimic_motor'] = nUtils.getObjectName(c_motor[0], phobostype = 'motor')
            props['mimic_multiplier'] = (obj.parent)['joint/mimic_multiplier']
            props['mimic_offset'] = (obj.parent)['joint/mimic_offset']
            break

    # try to derive the motor controller
    controllerobjs = [control for control in obj.children if control.phobostype == 'controller']
    if controllerobjs:
        controller = controllermodel.deriveController(controllerobjs[0])
    else:
        controller = None

    # assign the derived controller
    if controller:
        props['controller'] = controller['name']
    else:
        del props['controller']

    return props
