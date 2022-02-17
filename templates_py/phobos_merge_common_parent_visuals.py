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
from phobos.blender.utils.selection import selectObjects as select

arm = bpy.context.active_object
visuals = [v for v in bpy.context.selected_objects if v.phobostype == 'visual' and v.parent]
bonedict = {b.name: [] for b in bpy.context.object.data.bones}

for v in visuals:
    bonedict[v.parent_bone].append(v)

for b in bonedict:
    if bonedict[b]:
        select(bonedict[b], clear=True, active=0)
        bpy.ops.object.join()
