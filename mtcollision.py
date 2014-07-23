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
import mathutils
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
    bl_label = "Create collision objects for all selected Links"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name = 'coll_type',
        default = 'box',
        description = "collision type",
        items = mtdefs.geometrytypes)

    def execute(self, context):

        visuals = []
        for obj in bpy.context.selected_objects:
            if obj.MARStype == "visual":
                visuals.append(obj)
            obj.select = False

        if visuals == []:
            bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            return{'CANCELLED'}
        if not self.property_colltype == 'mesh': #TODO: copy mesh to collision object!
            for vis in visuals:
                nameparts = vis.name.split('_')
                if nameparts[0] == 'visual':
                    nameparts[0] = 'collision'
                collname = '_'.join(nameparts)
                bBox = vis.bound_box
                center = mtutility.calcBoundingBoxCenter(bBox)
                size = list(vis.dimensions)
                rotation = mathutils.Matrix.Identity(4)
                if self.property_colltype == 'cylinder':
                    axes = ('X', 'Y', 'Z')
                    long_side = axes[size.index(max(size))]
                    #xyequal = (size[0] - size[1])
                    height = max(size)
                    radii = [s for s in size if s is not height]
                    radius = max(radii)/2 if radii is not [] else height/2
                    size = (radius, height)
                    if long_side == 'X':
                        rotation = mathutils.Matrix.Rotation(math.pi/2, 4, 'Y')
                    elif long_side == 'Y':
                        rotation = mathutils.Matrix.Rotation(math.pi/2, 4, 'X')
                elif self.property_colltype == 'sphere':
                    size = max(size)/2
                rotation = (vis.matrix_world*rotation).to_euler()
                center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion()*center
                ob = mtutility.createPrimitive(collname, self.property_colltype, size,
                                               mtdefs.layerTypes["collision"], vis.data.materials[0].name, center,
                                               rotation)
                #TODO: apply rotation for moved cylinder object?
                ob.MARStype = "collision"
                ob["geometryType"] = self.property_colltype
                if vis.parent:
                    ob.select = True
                    bpy.ops.object.transform_apply(scale=True)
                    vis.parent.select = True
                    bpy.context.scene.objects.active = vis.parent
                    bpy.ops.object.parent_set()
        return{'FINISHED'}


