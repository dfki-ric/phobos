#!/usr/bin/python
# coding=utf-8

"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

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

File motors.py

Created on 23.08.2018

@author: Simon V. Reichel
"""


import bpy
import mathutils
from phobos import defs
from phobos.phoboslog import log
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils


def createMotor(motor, parentobj, origin=mathutils.Matrix()):
    """This function creates a new motor specified by its parameters.

    Args:
        motor(dict): phobos representation of the new motor.
        parentobj (bpy_types.Object): object to parent new motor to
        origin (mathutils.Matrix): new motors origin

    Returns:
        bpy.types.Object -- new motor object
    """
    layers = defs.layerTypes['motor']
    bUtils.toggleLayer(layers, value=True)

    # create motor object
    if motor['shape'].startswith('resource'):
        newmotor = bUtils.createPrimitive(
            motor['name'], 'box', [1, 1, 1], layers,
            plocation=origin.to_translation(), protation=origin.to_euler(),
            pmaterial=motor['material'], phobostype='motor')
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
            motor['name'], motor['shape'], motor['size'], layers,
            plocation=origin.to_translation(), protation=origin.to_euler(),
            pmaterial=motor['material'], phobostype='motor')

    # assign the parent if available
    if parentobj is not None:
        sUtils.selectObjects([newmotor, parentobj], clear=True, active=1)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')

    # set motor properties
    newmotor.phobostype = 'motor'
    newmotor.name = motor['name']
    newmotor['motor/type'] = motor['type']

    # write the custom properties to the motor
    eUtils.addAnnotation(newmotor, motor['props'], namespace='motor')

    # select the new motor
    sUtils.selectObjects([newmotor], clear=True, active=0)
    return newmotor


def deriveMotor(obj, jointdict=None):
    """Derives motor information from an object.

    Args:
        obj (bpy_types.Object): Blender object to derive the motor from
        jointdict (dict): phobos representation of the respective joint

    Returns:
        dict -- phobos representation of a motor
    """
    import phobos.model.models as models
    import phobos.model.controllers as controllermodel
    props = models.initObjectProperties(obj, phobostype='motor')

    # return None if no motor is attached (there will always be at least a name in the props)
    if len(props) < 2:
        return None

    # make sure the parent is a joint
    if not obj.parent or obj.parent.phobostype != 'link' or 'joint/type' not in obj.parent:
        log("Can not derive motor from {}. Insufficient requirements from parent object!".format(
            obj.name), 'ERROR')
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

    # TODO this will go to smurf
    # try:
    #     if props['type'] == 'PID':
    #         if 'limits' in joint:
    #             props['minValue'] = joint['limits']['lower']
    #             props['maxValue'] = joint['limits']['upper']
    #     elif props['type'] == 'DC':
    #         props['minValue'] = 0
    #         props['maxValue'] = props["maxSpeed"]
    # except KeyError:
    #     log("Missing data in motor " + obj.name + '. No motor created.', "WARNING")
    #     return None
    return props
