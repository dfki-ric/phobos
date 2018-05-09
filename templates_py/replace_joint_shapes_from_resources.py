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

