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
      : bpy.types.Object or list(bpy.types.Object)-- new motor object or a list of the new motor_obj
      : bpy.types.Object or list(bpy.types.Object)-- new motor object or a list of the new motor_obj
      and the new controller object

    """
    layers = defs.layerTypes['motor']
    bUtils.toggleLayer(layers, value=True)

    # create motor object
    if motor['shape'].startswith('resource'):
        newmotor = bUtils.createPrimitive(
            motor['name'],
            'box',
            [1, 1, 1],
            layers,
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
            motor['name'],
            motor['shape'],
            motor['size'],
            layers,
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
    newmotor.name = motor['name']
    defname = motor['defname']

    # write the custom properties to the motor
    eUtils.addAnnotation(newmotor, motor['props'], namespace='motor', ignore=['defname'])

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
