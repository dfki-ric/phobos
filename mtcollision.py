'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtcollision.py

Created on 7 Jan 2014

@author: Malte Langosz

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty
import math
import marstools.mtmaterials as mtmaterials
import marstools.mtutility as mtutility
import marstools.mtdefs as mtdefs
from mathutils import Vector

def register():
    print("Registering mtcollision...")


def unregister():
    print("Unregistering mtcollision...")

class CreateCollisionObjects(Operator):
    """Select n bodies to create collision objects for."""
    bl_idname = "object.create_collision_objects"
    bl_label = "Create collition objects for all selected Nodes"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        nodes = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "visual":
                nodes.append(obj)
            obj.select = False

        if nodes == []:
            bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            return{'CANCELLED'}
        parent = False
        for node in nodes:
            bBox = node.bound_box
            center = mtutility.calcBoundingBoxCenter(bBox)
            size = [0.0, 0.0, 0.0]
            size[0] = abs(2.0*(bBox[0][0] - center[0]))
            size[1] = abs(2.0*(bBox[0][1] - center[1]))
            size[2] = abs(2.0*(bBox[0][2] - center[2]))

            center = node.matrix_world.to_translation() + node.matrix_world.to_quaternion()*center

            ob = mtutility.createPrimitive('coll_'+node.name, 'box', (size[0], size[1], size[2]),
                                           mtdefs.layerTypes["collision"], 'joint', center,
                                           node.matrix_world.to_euler())
            ob.MARStype = "collision"
            ob["type"] = "box"
            if node.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                node.parent.select = True
                bpy.context.scene.objects.active = node.parent
                bpy.ops.object.parent_set()
        return{'FINISHED'}

class SetCollisionTypes(Operator):
    """Select n bodies to set collision type."""
    bl_idname = "object.set_collision_types"
    bl_label = "Set collition types for all selected Nodes"
    bl_options = {'REGISTER', 'UNDO'}

    collision_type = EnumProperty(
        name = "collision_type",
        default = "box",
        description = "type of the collision geometry",
        items = [tuple(['box']*3),
                 tuple(['sphere']*3),
                 tuple(['cylinder']*3),
                 tuple(['capsule']*3),
                 tuple(['mesh']*3)] #TODO: move this to mtdefs?
    )

    def execute(self, context):
        nodes = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "collision":
                nodes.append(obj)

        for node in nodes:
            node["type"] = self.collision_type

        return{'FINISHED'}
