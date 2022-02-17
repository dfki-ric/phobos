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

from phobos.blender.utils.selection import selectObjects, getImmediateChildren
from phobos.blender.model.motors import createMotor
from phobos.blender.utils.io import getDictFromYamlDefs


def derive_oldMotor(obj):
    """

    Args:
      obj: 

    Returns:

    """
    key_map = {'motor/type': 'motor/type', 'motor/name': 'motor/name'}

    new_motor = {}

    for oldProps, newProps in key_map.items():
        if oldProps in obj.keys():
            if oldProps == 'motor/type' and obj[oldProps] == 'PID':
                new_motor.update({newProps: 'generic_bldc'})
            elif oldProps == 'motor/type' and obj[oldProps] == 'DC':
                new_motor.update({newProps: 'generic_dc'})
            else:
                new_motor.update({newProps: obj[oldProps]})
            
    if not 'motor/name' in new_motor.keys():
        new_motor.update({'motor/name': obj.name + '_Motor'})

    return new_motor


def update_child(child, obj):
    """

    Args:
      child: 
      obj: 

    Returns:

    """
    motor_keys = ['motor/maxEffort', 'motor/maxSpeed']
    controller_map = {
        'controller/p': 'motor/p',
        'controller/i': 'motor/i',
        'controller/d': 'motor/d',
    }

    if isinstance(child, list):
        for children in child:
            update_child(children, obj)
    else:
        if child.phobostype == 'motor':
            child.name = obj.name + '_Motor'
            child['motor/name'] = obj.name
            for prop in motor_keys:
                try:
                    child[prop] = obj[prop]
                    del obj[prop]
                except:
                    pass
            if 'motor/type' in obj.keys():
                del obj['motor/type']

        elif child.phobostype == 'controller':
            child.name = obj.name + '_Controller'
            child['controller/name'] = obj.name
            for new, old in controller_map.items():
                try:
                    child[new] = obj[old]
                    del obj[old]
                except:
                    pass


selected_only = True

objectlist = bpy.context.selected_objects if selected_only else bpy.data.objects
for obj in objectlist:
    if obj.phobostype == 'link':
        motor_found = any('motor' in key for key in obj.keys())
        if motor_found:
            new_motor_dict = derive_oldMotor(obj)
            motor_dict = getDictFromYamlDefs(
                'motor', new_motor_dict['motor/type'], new_motor_dict['motor/name']
            )
            new_objects = createMotor(motor_dict, obj, origin=obj.matrix_world, addcontrollers=True)
            update_child(new_objects, obj)
