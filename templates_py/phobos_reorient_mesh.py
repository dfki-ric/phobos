import bpy

def compareOrientation(obj, step):
    before = sum(obj.dimensions)
    bpy.ops.transform.rotate(value=step, axis=(0, 0, 1))
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    after = sum(obj.dimensions)
    return before, after

def minimizeMeshDimensions(obj, direction, step, epsilon):
    stepsum = 0
    while (True):
        before, after = compareOrientation(obj, direction*step)
        if before < after:
            #bpy.ops.transform.rotate(value=-1.0*direction*step, axis=(0, 0, 1))
            #bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
            break
        else:
            stepsum += direction*step
    step = step / 2
    if step > epsilon:
        print(stepsum)
        stepsum += minimizeMeshDimensions(obj, -direction, step, epsilon)
    return stepsum

print('meeerp')
correction = minimizeMeshDimensions(bpy.context.active_object, 1.0, 0.3, 0.00000001)
bpy.ops.transform.rotate(value=-correction, axis=(0, 0, 1))
