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
import phobos.blender.utils.io as ioUtils
import phobos.blender.utils.naming as nUtils
from phobos.blender.phoboslog import log

joints = [obj for obj in bpy.context.selected_objects if obj.phobostype == 'link']
jointtypes = set()
for joint in joints:
    try:
        jointtypes.add(joint['joint/type'])
    except KeyError:
        log("No joint type defined for link object: " + joint.name, 'INFO')

resources = ioUtils.importResources((('joint', jointtype) for jointtype in jointtypes))

for joint in joints:
    try:
        joint.pose.bones[0].custom_shape = bpy.data.scenes['resources'].objects[
            nUtils.addNamespaceToName('joint' + '_' + joint['joint/type'], 'resource')
        ]
    except KeyError:
        log("No joint type defined for link object: " + joint.name, 'INFO')
