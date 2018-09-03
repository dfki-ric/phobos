# TODO SHEBANG

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
from phobos.utils.selection import selectObjects as select
import phobos.utils.editing as eUtils

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
