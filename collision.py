"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File collision.py

Created on 7 Jan 2014

@author: Kai von Szadkowski
"""

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty, EnumProperty, BoolVectorProperty
import math
from mathutils import Vector, Matrix
from . import materials
from . import utility
from . import defs
from phobos.logging import *


def register():
    print("Registering collision...")


def unregister():
    print("Unregistering collision...")

class CreateCollisionObjects(Operator):
    """Select n bodies to create collision objects for."""
    bl_idname = "object.create_collision_objects"
    bl_label = "Create collision objects for all selected Links"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name='coll_type',
        default='box',
        description="collision type",
        items=defs.geometrytypes)

    def execute(self, context):

        startLog(self)
        visuals = []
        for obj in bpy.context.selected_objects:
            if obj.phobostype == "visual":
                visuals.append(obj)
            obj.select = False

        if not visuals:
            #bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            log("Not enough bodies selected.", "ERROR")
            return{'CANCELLED'}
        for vis in visuals:
            nameparts = vis.name.split('_')
            if nameparts[0] == 'visual':
                nameparts[0] = 'collision'
            collname = '_'.join(nameparts)
            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"
            bBox = vis.bound_box
            center = utility.calcBoundingBoxCenter(bBox)
            rotation = Matrix.Identity(4)
            size = list(vis.dimensions)
            if self.property_colltype in ['cylinder', 'capsule']:
                axes = ('X', 'Y', 'Z')
                long_side = axes[size.index(max(size))]
                #xyequal = (size[0] - size[1])
                length = max(size)
                radii = [s for s in size if s != length]
                radius = max(radii)/2 if radii != [] else length/2
                size = (radius, length)
                if long_side == 'X':
                    rotation = Matrix.Rotation(math.pi/2, 4, 'Y')
                elif long_side == 'Y':
                    rotation = Matrix.Rotation(math.pi/2, 4, 'X')
                #FIXME: apply rotation for moved cylinder object?
            elif self.property_colltype == 'sphere':
                size = max(size)/2
            rotation_euler = (vis.matrix_world*rotation).to_euler()
            center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion()*center
            if self.property_colltype != 'capsule':
                ob = utility.createPrimitive(collname, self.property_colltype, size,
                                               defs.layerTypes['collision'], materialname, center,
                                               rotation_euler)
            elif self.property_colltype == 'capsule':
                length = max(length-2*radius, 0.001) #prevent length from turning negative
                size = (radius, length)
                zshift = length/2
                ob = utility.createPrimitive(collname, 'cylinder', size,
                               defs.layerTypes['collision'], materialname, center,
                               rotation_euler)
                sph1 = utility.createPrimitive('tmpsph1', 'sphere', radius,
                               defs.layerTypes['collision'], materialname, center + rotation * Vector((0,0,zshift)),
                               rotation_euler)
                sph2 = utility.createPrimitive('tmpsph2', 'sphere', radius,
                               defs.layerTypes['collision'], materialname, center - rotation * Vector((0,0,zshift)),
                               rotation_euler)
                utility.selectObjects([ob, sph1, sph2], True, 0)
                bpy.ops.object.join()
                ob['length'] = length
                ob['radius'] = radius
            elif self.property_colltype == 'mesh':
                pass
                #TODO: copy mesh!!
            ob.phobostype = 'collision'
            ob['geometry/type'] = self.property_colltype
            if vis.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                vis.parent.select = True
                bpy.context.scene.objects.active = vis.parent
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                #ob.parent_type = vis.parent_type
                #ob.parent_bone = vis.parent_bone
        endLog()
        return{'FINISHED'}


    class SetCollisionGroupOperator(Operator):
        """SetCollisionGroupOperator"""
        bl_idname = "object.phobos_set_collision_group"
        bl_label = "Sets the collision group of the selected object(s)."
        bl_options = {'REGISTER', 'UNDO'}

        groups = BoolVectorProperty(
            name='collision groups',
            size=20,
            subtype='LAYER',
            default=(False,)*20,
            description='collision groups')

        @classmethod
        def poll(self, context):
            for obj in context.selected_objects:
                if obj.phobostype == 'collision':
                    return True
            return False

        def invoke(self, context, event):
            try:
                self.groups = context.active_object.rigid_body.collision_groups
            except AttributeError:
                pass  # TODO: catch properly
            return self.execute(context)

        def execute(self, context):
            active_object = context.active_object
            for obj in context.selected_objects:
                if obj.phobostype == 'collision':
                    try:
                        obj.rigid_body.collision_groups = self.groups
                    except AttributeError:
                        context.scene.objects.active = obj
                        bpy.ops.rigidbody.object_add(type='ACTIVE')
                        obj.rigid_body.collision_groups = self.groups
            context.scene.objects.active = active_object
            return {'FINISHED'}


