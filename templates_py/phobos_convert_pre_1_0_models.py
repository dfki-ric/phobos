import bpy
import phobos.utils.selection as sUtils
import phobos.utils.io as ioUtils

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
    elif (obj.phobostype == 'inertial' and 'inertial/mass' in obj and 'inertial/inertia' in obj
          and delete_link_inertials):
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

        if sUtils.isRoot(obj):
            bpy.context.scene.objects.active = obj
            bpy.ops.phobos.set_model_root()

# update joint shapes for whole model
joints = [obj for obj in objectlist if 'joint/type' in obj]
for joint in joints:
    resource_obj = ioUtils.getResource(['joint', joint['joint/type']])
    joint.pose.bones[0].custom_shape = resource_obj
