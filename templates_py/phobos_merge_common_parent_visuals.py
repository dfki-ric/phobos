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

