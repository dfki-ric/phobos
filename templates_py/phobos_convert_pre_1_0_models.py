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
import phobos.blender.utils.selection as sUtils
import phobos.blender.utils.io as ioUtils

# Things this script does not cover:
#    - updating old sensors to new sensors (delete sensor/* properties and replace them with the
#    right sensor object)
#    - updating old motors to new motors (delete motor/* properties from links and insert motor
#    objects instead)

# -----------------------------------------------------------------
# configuration
# -----------------------------------------------------------------
# remove existing link inertial objects? (no longer required if derived from helper objects)
delete_link_inertials = True
# convert only selected objects or all objects in .blend file?
selected_only = False
# -----------------------------------------------------------------

objects_to_be_deleted = []
objectlist = bpy.context.selected_objects if selected_only else bpy.data.objects
for obj in objectlist:
    if obj.phobostype == 'inertial' and 'mass' in obj and 'inertia' in obj:
        obj['inertial/mass'] = obj['mass']
        del obj['mass']
        obj['inertial/inertia'] = obj['inertia']
        del obj['inertia']
    elif (
        obj.phobostype == 'inertial'
        and 'inertial/mass' in obj
        and 'inertial/inertia' in obj
        and delete_link_inertials
    ):
        objects_to_be_deleted.append(obj)

# delete objects designated for deletion
sUtils.selectObjects(objects_to_be_deleted)
bpy.ops.object.delete(use_global=True)

# remove motor limits custom properties
for obj in objectlist:
    if 'motor/limit' in obj:
        del obj['motor/limit']
    elif 'motor/limits' in obj:
        del obj['motor/limits']

# move modelname to model/name and reassign root
for obj in objectlist:
    if 'modelname' in obj:
        obj['model/name'] = obj['modelname']
        del obj['modelname']

        if 'version' in obj:
            obj['model/version'] = obj['version']
            del obj['version']

        if sUtils.isRoot(obj):
            bpy.context.view_layer.objects.active = obj
            bpy.ops.phobos.set_model_root()

# update joint shapes for whole model
joints = [obj for obj in objectlist if 'joint/type' in obj]
for joint in joints:
    resource_obj = ioUtils.getResource(['joint', joint['joint/type']])
    joint.pose.bones[0].custom_shape = resource_obj
