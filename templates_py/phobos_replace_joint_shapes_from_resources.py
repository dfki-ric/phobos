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
import phobos.utils.io as ioUtils
import phobos.utils.naming as nUtils
from phobos.phoboslog import log

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
            nUtils.addNamespaceToName('joint' + '_' + joint['joint/type'], 'resource')]
    except KeyError:
        log("No joint type defined for link object: " + joint.name, 'INFO')

