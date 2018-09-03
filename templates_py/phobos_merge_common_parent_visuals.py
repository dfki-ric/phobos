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

arm = bpy.context.active_object
visuals = [v for v in bpy.context.selected_objects if v.phobostype == 'visual' and v.parent]
bonedict = {b.name: [] for b in bpy.context.object.data.bones}

for v in visuals:
    bonedict[v.parent_bone].append(v)

for b in bonedict:
    if bonedict[b]:
        select(bonedict[b], clear=True, active=0)
        bpy.ops.object.join()

