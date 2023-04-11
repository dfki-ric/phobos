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
import phobos.blender.utils.editing as eUtils

root = bpy.context.active_object
links = [link for link in bpy.context.selected_objects if link.phobostype == 'link']
visuals = [v for v in bpy.context.selected_objects if v.phobostype == 'visual' and v.parent]

# rename all bones so they are not all named 'Bone' in the joint armature
for link in links:
    select([link], clear=True, active=0)
    link.data.bones[0].name = link.name
    bpy.ops.object.mode_set(mode='EDIT')
    print(link.name, len(link.data.bones), len(link.data.edit_bones))
    link.data.edit_bones[0].name = link.name

    bpy.ops.object.mode_set(mode='OBJECT')


# save list of all parent joints for all visuals
vparents = {}
lparents = {}
for v in visuals:
    vparents[v.name] = v.parent.name
for l in links:
    try:
        lparents[l.name] = l.parent.name
    except AttributeError:
        pass  # root link

select(visuals, clear=True, active=0)
bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
select(links, clear=True, active=links.index(root))
bpy.context.scene.objects.active = root
bpy.ops.object.join()

bpy.ops.object.mode_set(mode='EDIT')
for key, value in lparents.items():
    root.data.edit_bones[key].parent = root.data.edit_bones[value]
bpy.ops.object.mode_set(mode='OBJECT')

for v in visuals:
    select([root], clear=True, active=0)
    bpy.ops.object.join()
    bpy.ops.object.mode_set(mode='EDIT')
    root.data.edit_bones.active = root.data.edit_bones[vparents[v.name]]
    bpy.ops.object.mode_set(mode='OBJECT')
    eUtils.parentObjectsTo(v, root, 1)
