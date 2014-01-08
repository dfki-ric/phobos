'''
Created on 7 Jan 2014

@author: kavonszadkowski
'''

#The following script places green haaelper sphere objects at the positions of all selected objects and fills in name and other properties, assuming that the selected objects are joints
#NOTE: For this script to work, you need to make all involved layers visible (probably 0, 1, 2)

import bpy
from bpy.types import Operator
import marstools.mtmaterials as mtmaterials

def register():
    print("Registering mtjoints...")


def unregister():
    print("Unregistering mtjoints...")



class deriveJointSpheresOperator(Operator):
    """DeriveJointSpheresOperator"""
    bl_idname = "object.derive_joint_spheres"
    bl_label = "Creates Joint Helper Objects for all Joints"
    bl_options = {'REGISTER', 'UNDO'}

    js_layers = 20*[False]
    js_layers[2] = True
    for joint in bpy.context.selected_objects:
        # create ball
        bpy.ops.mesh.primitive_uv_sphere_add(size = 0.01,
                                      layers = js_layers)
    #                                  location = joint.location)
                                     # rotation = joint.rotation_euler)
        ball = bpy.context.object
        ball.name = 'joint_sphere_' + joint.name
        ball.data.materials.append(mtmaterials.materials['joint_sphere'])
        ball.parent = joint.parent
        ball.select = True
        bpy.context.scene.objects.active = joint.parent
        bpy.ops.object.parent_set() #makes active object parent of selected object
        ball.location = joint.location
        ball.rotation_euler = joint.rotation_euler

        #modify limb
        print(joint.name)
        print("    ", joint['node2'])
        print("")
        limb = bpy.context.scene.objects[joint['node2']]
        limb.select = True
        bpy.context.scene.objects.active = ball
        bpy.ops.object.parent_set()

        #modify joint
        joint['anchor'] = "node2"
        joint['node2'] = ball.name
        ball["coll_bitmask"] = 0
        ball["type"] = "body"
        ball["mass"] = 0
