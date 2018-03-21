#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.editing
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators to manipulate blender objects

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert, Simon Reichel

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

File operators/editing.py

@author: Kai von Szadkowski, Stefan Rahms, Simon Reichel
"""

import math
import os
import yaml
import inspect
import sys
from datetime import datetime

import bpy
import mathutils
from bpy.types import Operator
from bpy.props import (BoolProperty, IntProperty, StringProperty, EnumProperty,
                       FloatProperty, FloatVectorProperty, BoolVectorProperty)

import phobos.defs as defs
import phobos.display as display
import phobos.model.inertia as inertialib
import phobos.utils.selection as sUtils
import phobos.utils.general as gUtils
import phobos.utils.blender as bUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.utils.validation as vUtils
import phobos.model.joints as joints
import phobos.model.links as modellinks
import phobos.model.sensors as sensors
from phobos.phoboslog import log


class SafelyRemoveObjectsFromSceneOperator(Operator):
    """Removes all selected objects from scene, warning if they are deleted"""
    bl_idname = "phobos.safely_remove_objects_from_scene"
    bl_label = "Safely Remove Objects From Scene"
    bl_options = {'UNDO'}

    def execute(self, context):
        objs_to_delete = [o for o in context.selected_objects if len(o.users_scene) > 1]
        objs_to_keep = [o for o in context.selected_objects if len(o.users_scene) == 1]
        sUtils.selectObjects(objs_to_delete, clear=True)
        bpy.ops.object.delete()
        sUtils.selectObjects(objs_to_keep, clear=True)
        if len(objs_to_keep) > 0:
            log('Some objects were not removed as they are unique to this scene.', 'WARNING')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


class MoveToSceneOperator(Operator):
    """Moves all selected objects to (new) scene"""
    bl_idname = "phobos.move_to_scene"
    bl_label = "Move To Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def getSceneEnumProperty(self, context):
        return bUtils.compileEnumPropertyList(bpy.data.scenes.keys())

    scenename = StringProperty(name='Scene Name',
                               default='new',
                               description='Name of the scene to which to add selection')

    scene = EnumProperty(name='Scene',
                         items=getSceneEnumProperty,
                         description='List of available scenes')

    move = BoolProperty(name='Move', default=False,
                        description="Remove selected objects from active scene")

    init = BoolProperty(name='Init', default=True,
                        description="Init new scene with items from active scene")

    new = BoolProperty(name='New', default=True,
                        description="Create new scene to move items to")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def check(self, context):
        if not self.new:
            self.scenename = self.scene
        return True

    def draw(self, context):
        self.layout.prop(self, 'scene')
        self.layout.prop(self, 'new')
        if self.new:
            self.layout.prop(self, 'scenename', text="Scene Name (if new)")
            self.layout.prop(self, 'init')
        self.layout.prop(self, 'move')

    def execute(self, context):
        moveobjs = context.selected_objects
        oldscene = context.scene
        if self.new and self.scenename not in bpy.data.scenes:
            bpy.data.scenes.new(name=self.scenename)
            # copy all objects from active scene if new scene is created
            if self.init:
                for obj in oldscene.objects:
                    try:
                        bpy.data.scenes[self.scenename].objects.link(obj)
                    except RuntimeError as e:
                        log(str(e), 'WARNING')
        # in any case, make sure to move selected objects objects to scene
        for obj in moveobjs:
            try:
                bpy.data.scenes[self.scenename].objects.link(obj)
            except RuntimeError as e:
                log(str(e), 'WARNING')
        # remove objects from active scene
        if self.move:
            for obj in moveobjs:
                try:
                    oldscene.objects.unlink(obj)
                except RuntimeError as e:
                    log(str(e), 'WARNING')
        bpy.context.screen.scene = bpy.data.scenes[self.scenename]
        bpy.data.scenes[self.scenename].update()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


class SortObjectsToLayersOperator(Operator):
    """Sort all selected objects to their according layers"""
    bl_idname = "phobos.sort_objects_to_layers"
    bl_label = "Sort Objects to Layers"
    bl_options = {'UNDO'}

    def execute(self, context):
        for obj in context.selected_objects:
            if obj.phobostype != 'undefined':
                layers = 20 * [False]
                layers[defs.layerTypes[obj.phobostype]] = True
                obj.layers = layers
            else:
                log("The phobostype of object '" + obj.name + "' is" + "undefined", "ERROR")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


class AddKinematicChainOperator(Operator):
    """Add a kinematic chain between two selected objects"""
    bl_idname = "phobos.add_kinematic_chain"
    bl_label = "Add Kinematic Chain"
    bl_options = {'REGISTER', 'UNDO'}

    chainname = StringProperty(
        name='Chain Name',
        default='new_chain',
        description='Name of the chain to be created')

    def execute(self, context):
        endobj = context.active_object
        # pick first nonactive object as start
        for obj in context.selected_objects:
            if obj is not context.active_object:
                startobj = obj
                break

        # add chain properties to startobj
        if 'startChain' not in startobj:
            startobj['startChain'] = [self.chainname]
        else:
            namelist = startobj['startChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            startobj['startChain'] = namelist

        # add chain properties to endobj
        if 'endChain' not in endobj:
            endobj['endChain'] = [self.chainname]
        else:
            namelist = endobj['endChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            endobj['endChain'] = namelist
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) == 2


class SetXRayOperator(Operator):
    """Show the selected/chosen objects via X-ray"""
    bl_idname = "phobos.set_xray"
    bl_label = "X-Ray Vision"
    bl_options = {'REGISTER', 'UNDO'}

    objects = EnumProperty(
        name="Objects",
        default='selected',
        items=(('all',) * 3, ('selected',) * 3, ('by name',) * 3) + defs.phobostypes,
        description="Show objects via x-ray")

    show = BoolProperty(
        name="Enable X-Ray",
        default=True,
        description="Enable or disable X-Ray")

    namepart = StringProperty(
        name="Name Contains",
        default="",
        description="Part of a name for objects to be selected in 'by name' mode")

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT' or context.mode == 'POSE'

    def draw(self, context):
        layout = self.layout
        layout.label(text="Select items for X-ray view")

        layout.prop(self, "objects")
        layout.prop(self, "show")

        # show name text field only when changing by name
        if self.objects == 'by name':
            layout.prop(self, "namepart")

    def execute(self, context):
        # pick objects to change
        if self.objects == 'all':
            objlist = bpy.data.objects
        elif self.objects == 'selected':
            objlist = context.selected_objects
        elif self.objects == 'by name':
            objlist = [obj for obj in bpy.data.objects if obj.name.find(self.namepart) >= 0]
        else:
            objlist = [obj for obj in bpy.data.objects if obj.phobostype == self.objects]

        # change xray setting
        for obj in objlist:
            obj.show_x_ray = self.show
        return {'FINISHED'}


class SetPhobosType(Operator):
    """Change phobostype of selected object(s)"""
    bl_idname = "phobos.set_phobostype"
    bl_label = "Set Phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    phobostype = EnumProperty(
        items=defs.phobostypes,
        name="Phobostype",
        default="undefined",
        description="Phobostype")

    def execute(self, context):
        """Change phobostype of all selected objects.
        """
        if self.phobostype == 'undefined':
            log("Setting phobostype 'undefined' for selected objects.", 'WARNING')

        for obj in context.selected_objects:
            obj.phobostype = self.phobostype
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.selected_objects

    def invoke(self, context, event):
        if context.object:
            self.phobostype = context.object.phobostype
        return self.execute(context)


class BatchEditPropertyOperator(Operator):
    """Edit custom property of selected object(s)"""
    bl_idname = "phobos.batch_property"
    bl_label = "Edit Custom Property"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = StringProperty(
        name="Name",
        default="",
        description="Custom property name")

    property_value = StringProperty(
        name="Value",
        default="",
        description="Custom property value")

    def execute(self, context):
        value = gUtils.parse_number(self.property_value)
        # delete property when value is empty
        if value == '':
            for obj in context.selected_objects:
                if self.property_name in obj.keys():
                    del(obj[self.property_name])
        # change property
        else:
            for obj in context.selected_objects:
                obj[self.property_name] = value
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and len(context.selected_objects) > 0 and ob.mode == 'OBJECT'


class CreateInterfaceOperator(Operator):
    """Create interface and optionally attach to parent"""
    bl_idname = "phobos.create_interface"
    bl_label = "Create Interface"
    bl_options = {'REGISTER', 'UNDO'}

    interface_name = StringProperty(
        name='name',
        default='interface')

    interface_type = StringProperty(
        name='type',
        default='default')

    interface_direction = EnumProperty(
        name='direction',
        items=bUtils.compileEnumPropertyList(('outgoing', 'incoming', 'bidirectional')),
        default='outgoing')

    all_selected = BoolProperty(
        name='all selected',
        default=False
    )

    scale = FloatProperty(
        name='scale',
        default=1.0
    )

    def execute(self, context):
        ifdict = {'type': self.interface_type,
                  'direction': self.interface_direction,
                  'name': self.interface_name,
                  'scale': self.scale}
        if self.all_selected:
            for link in [obj for obj in context.selected_objects if obj.phobostype == 'link']:
                ifdict['parent'] = link
                ifdict['name'] = link.name + '_' + self.interface_name
                eUtils.createInterface(ifdict, link)
        else:
            eUtils.createInterface(ifdict, context.object)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        if context.object:
            return context.object.mode == 'OBJECT'
        else:
            return True


class CopyCustomProperties(Operator):
    """Copy custom properties of selected object(s)"""
    bl_idname = "phobos.copy_props"
    bl_label = "Copy Custom Properties"
    bl_options = {'REGISTER', 'UNDO'}

    empty_properties = BoolProperty(
        name='empty',
        default=False,
        description="empty properties?")

    def execute(self, context):
        slaves = context.selected_objects
        master = context.active_object
        slaves.remove(master)
        props = bUtils.cleanObjectProperties(dict(master.items()))
        for obj in slaves:
            if self.empty_properties:
                for key in obj.keys():
                    del (obj[key])
            for key in props.keys():
                obj[key] = props[key]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        obs = context.selected_objects
        ob = context.active_object
        return len(obs) > 0 and ob is not None and ob.mode == 'OBJECT'


class RenameCustomProperty(Operator):
    """Rename custom property of selected object(s)"""
    bl_idname = "phobos.rename_custom_property"
    bl_label = "Rename Custom Property"
    bl_options = {'REGISTER', 'UNDO'}

    find = StringProperty(
        name="Find Property Name",
        default='',
        description="Name to be searched for")

    replace = StringProperty(
        name="Replacement Name",
        default='',
        description="New name to be replaced with")

    overwrite = BoolProperty(
        name='Overwrite Existing Properties',
        default=False,
        description="If a property of the specified replacement name exists, overwrite it?"
    )

    def execute(self, context):
        objs = filter(lambda e: self.find in e, context.selected_objects)
        if self.replace != "":
            for obj in objs:
                if self.replace in obj and not self.overwrite:
                    log("Property '" + self.replace + "' already present in" +
                        "object '" + obj.name + "'", "ERROR")
                else:
                    obj[self.replace] = obj[self.find]
                    del obj[self.find]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetGeometryType(Operator):
    """Edit geometry type of selected object(s)"""
    bl_idname = "phobos.define_geometry"
    bl_label = "Define Geometry"
    bl_options = {'REGISTER', 'UNDO'}

    geomType = EnumProperty(
        items=defs.geometrytypes,
        name="Type",
        default="box",
        description="Phobos geometry type")

    def execute(self, context):
        objs = filter(lambda e: "phobostype" in e, context.selected_objects)
        for obj in objs:
            if obj.phobostype == 'collision' or obj.phobostype == 'visual':
                obj['geometry/type'] = self.geomType
            else:
                log("The object '" + obj.name + "' is no collision or visual.", "INFO")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SmoothenSurfaceOperator(Operator):
    """Smoothen surface of selected objects"""
    bl_idname = "phobos.smoothen_surface"
    bl_label = "Smoothen Surface"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        objs = [obj for obj in context.selected_objects if obj.type == "MESH"]
        i = 1
        for obj in objs:
            context.scene.objects.active = obj
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.normals_make_consistent()
            bpy.ops.mesh.mark_sharp(clear=True)
            bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.shade_smooth()
            bpy.ops.object.modifier_add(type='EDGE_SPLIT')
            display.setProgress(i/len(context.selected_objects))
            i += 1
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class EditInertialData(Operator):
    """Edit inertia of selected object(s)"""
    bl_idname = "phobos.edit_inertial_data"
    bl_label = "Edit Mass/Inertia"
    bl_options = {'REGISTER', 'UNDO'}

    inertiavector = FloatVectorProperty(
        name="Inertia Vector",
        default=[0, 0, 0, 0, 0, 0],
        subtype='NONE',
        size=6,
        description="Set inertia for a link"
    )

    mass = FloatProperty(
        name='Mass',
        default=0.001,
        description="Mass (of active object) in kg")

    def invoke(self, context, event):
        # read initial parameter values from active object is possible
        if context.active_object:
            if 'inertial/mass' in context.active_object:
                self.mass = context.active_object['inertial/mass']
            if 'inertial/inertia' in context.active_object:
                self.inertiavector = mathutils.Vector(context.active_object['inertial/inertia'])
        return self.execute(context)

    def execute(self, context):
        objs = [obj for obj in context.selected_objects if obj.phobostype == "inertial"]
        for obj in objs:
            obj['inertial/mass'] = self.mass
            obj['inertial/inertia'] = self.inertiavector
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return bool([obj for obj in context.selected_objects if obj.phobostype == 'inertial'])


class GenerateInertialObjectsOperator(Operator):
    """Creates inertial object(s) to add mass and inertia data to links"""
    bl_idname = "phobos.generate_inertial_objects"
    bl_label = "Define Mass/Inertia"
    bl_options = {'REGISTER', 'UNDO'}

    mass = FloatProperty(
        name='Mass',
        default=0.001,
        description="Mass (of active object) in kg")

    inertiavector = FloatVectorProperty(
        name="Inertia Vector",
        default=[0, 0, 0, 0, 0, 0],
        subtype='NONE',
        size=6,
        description="Set inertia for a link"
    )

    derive_inertia_from_geometry = BoolProperty(
        name="Calculate inertia from geometry ",
        default=True,
        description="Derive inertia value(s) from geometry of visual or collision objects."
    )

    clear = BoolProperty(
        name="Clear existing inertial objects",
        default=True,
        description="Clear existing inertial objects of selected links."
    )

    # this is only necessary to prevent crashing
    run = BoolProperty(
        name="Run operator with settings",
        default=False,
        description="Confirm to run the operator with these settings"
    )

    # TODO: running this operator as dialog leads Blender to crash
    def invoke(self, context, event):
        self.run = False
        return self.execute(context)
    #     return context.window_manager.invoke_props_dialog(self, width=300)
    #
    # def check(self, context):
    #     return True

    def draw(self, context):
        layout = self.layout
        if not self.run:
            layout.prop(self, 'clear')
            geometric_objects = [obj for obj in context.selected_objects
                                 if obj.phobostype in ['visual', 'collision']]
            if geometric_objects:
                layout.prop(self, 'derive_inertia_from_geometry')
            else:
                self.derive_inertia_from_geometry = False
            if not self.derive_inertia_from_geometry:
                layout.prop(self, 'inertiavector')
            layout.prop(self, 'mass')
            layout.separator()
            layout.prop(self, 'run', toggle=True)
        else:
            layout.label('Done')

    def execute(self, context):
        if self.run:
            # store previously selected objects
            selection = context.selected_objects

            geometric_objects = [obj for obj in selection
                                 if obj.phobostype in ['visual', 'collision']]
            if self.derive_inertia_from_geometry:
                link_objects = list(set([obj.parent for obj in selection
                                         if obj.phobostype in ['visual', 'collision']]))
            else:
                link_objects = [obj for obj in selection if obj.phobostype == 'link']
            inertial_objects = [obj for link in link_objects for obj in link.children
                                if obj.phobostype == 'inertial']

            new_inertial_objects = []
            if self.derive_inertia_from_geometry:
                i = 1
                for obj in geometric_objects:
                    inertialdict = {'mass': self.mass,
                                    'inertia': inertialib.calculateInertia(obj, self.mass),
                                    'pose': {'translation': obj.matrix_local.to_translation()}}
                    newinertial = inertialib.createInertial(inertialdict, obj)
                    if newinertial:
                        new_inertial_objects.append(newinertial)
                    display.setProgress(i / len(geometric_objects))
                    i += 1
            else:
                i = 1
                for obj in link_objects:
                    inertialdict = {'mass': self.mass,
                                    'inertia': self.inertiavector,
                                    'pose': {'translation': mathutils.Vector((0.0, 0.0, 0.0))}}
                    newinertial = inertialib.createInertial(inertialdict, obj)
                    if newinertial:
                        new_inertial_objects.append(newinertial)
                    display.setProgress(i / len(link_objects))
                    i += 1

            if new_inertial_objects:  # select the new inertialobjs if created
                sUtils.selectObjects(new_inertial_objects, clear=True)

            if self.clear:
                for obj in inertial_objects:
                    bpy.data.objects.remove(obj)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return (context.selected_objects is not None and
                bool({'visual', 'collision', 'inertial', 'link'} &
                     set([o.phobostype for o in context.selected_objects])))


class EditYAMLDictionary(Operator):
    """Edit object dictionary as YAML"""
    bl_idname = 'phobos.edityamldictionary'
    bl_label = "Edit Object Dictionary"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        ob = context.active_object
        textfilename = ob.name + datetime.now().strftime("%Y%m%d_%H:%M")
        variablename = ob.name.translate({ord(c): "_" for c in "!@#$%^&*()[]{};:,./<>?\|`~-=+"}) \
                       + "_data"
        tmpdict = dict(ob.items())

        # write object properties to short python script
        for key in tmpdict:
            # transform Blender id_arrays into lists
            if hasattr(tmpdict[key], 'to_list'):
                tmpdict[key] = list(tmpdict[key])
        contents = [variablename + ' = """',
                    yaml.dump(bUtils.cleanObjectProperties(tmpdict),
                              default_flow_style=False) + '"""\n',
                    "# ------- Hit 'Run Script' to save your changes --------",
                    "import yaml", "import bpy",
                    "tmpdata = yaml.load(" + variablename + ")",
                    "for key in dict(bpy.context.active_object.items()):",
                    "   del bpy.context.active_object[key]",
                    "for key, value in tmpdata.items():",
                    "    bpy.context.active_object[key] = value",
                    "bpy.ops.text.unlink()"
                    ]

        # show python script to user
        bUtils.createNewTextfile(textfilename, '\n'.join(contents))
        bUtils.openScriptInEditor(textfilename)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class CreateCollisionObjects(Operator):
    """Create collision objects for all selected visual objects"""
    bl_idname = "phobos.create_collision_objects"
    bl_label = "Create Collision Object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name='Collision Type',
        default='box',
        description="Collision type",
        items=defs.geometrytypes)

    def execute(self, context):
        visuals = []
        collisions = []

        # find all selected visual objects
        for obj in context.selected_objects:
            if obj.phobostype == "visual":
                visuals.append(obj)
            obj.select = False

        if not visuals:
            log("No visual objects selected.", "ERROR", self)
            return {'CANCELLED'}

        # create collision objects for each visual
        for vis in visuals:
            # build object names
            nameparts = vis.name.split('_')
            if nameparts[0] == 'visual':
                nameparts[0] = 'collision'
            collname = '_'.join(nameparts)
            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"

            # get bounding box
            bBox = vis.bound_box
            center = gUtils.calcBoundingBoxCenter(bBox)
            rotation = mathutils.Matrix.Identity(4)
            size = list(vis.dimensions)

            # calculate size for cylinder, capsule or sphere
            if self.property_colltype in ['cylinder', 'capsule']:
                axes = ('X', 'Y', 'Z')
                long_side = axes[size.index(max(size))]
                length = max(size)
                radii = [s for s in size if s != length]
                radius = max(radii) / 2 if radii != [] else length / 2
                size = (radius, length)

                # rotate cylinder/capsule to match longest side
                if long_side == 'X':
                    rotation = mathutils.Matrix.Rotation(math.pi / 2, 4, 'Y')
                elif long_side == 'Y':
                    rotation = mathutils.Matrix.Rotation(math.pi / 2, 4, 'X')
                    # FIXME: apply rotation for moved cylinder object?

            elif self.property_colltype == 'sphere':
                size = max(size) / 2

            # combine bbox center with the object transformation
            center = (vis.matrix_world * mathutils.Matrix.Translation(center)).to_translation()
            # combine center with optional rotation (cylinder and capsule) and object transformation
            rotation_euler = (vis.matrix_world * rotation *
                              mathutils.Matrix.Translation(center)).to_euler()

            # create Mesh
            if self.property_colltype != 'capsule' and self.property_colltype != 'mesh':
                ob = bUtils.createPrimitive(collname, self.property_colltype, size,
                                            defs.layerTypes['collision'], materialname, center,
                                            rotation_euler)
            elif self.property_colltype == 'capsule':
                # TODO reimplement capsules
                # prevent length from turning negative
                length = max(length - 2 * radius, 0.001)
                size = (radius, length)
                zshift = length / 2
                tmpsph1_location = center + rotation_euler.to_matrix().to_4x4() * mathutils.Vector((0,0,zshift))
                tmpsph2_location = center - rotation_euler.to_matrix().to_4x4() * mathutils.Vector((0,0,zshift))

                # create cylinder and spheres and join them
                ob = bUtils.createPrimitive(collname, 'cylinder', size,
                                            defs.layerTypes['collision'], materialname, center,
                                            rotation_euler)
                sph1 = bUtils.createPrimitive('tmpsph1', 'sphere', radius,
                                              defs.layerTypes['collision'], materialname,
                                              tmpsph1_location,
                                              rotation_euler)
                sph2 = bUtils.createPrimitive('tmpsph2', 'sphere', radius,
                                              defs.layerTypes['collision'], materialname,
                                              tmpsph2_location,
                                              rotation_euler)
                sUtils.selectObjects([ob, sph1, sph2], True, 0)
                bpy.ops.object.join()

                # assign capsule properties
                ob['geometry/length'] = length
                ob['geometry/radius'] = radius
                ob['sph1_location'] = tmpsph1_location
                ob['sph2_location'] = tmpsph2_location
            elif self.property_colltype == 'mesh':
                # FIXME: simply turn this into object.duplicate?
                bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked": False, "mode": 'TRANSLATION'},
                                              TRANSFORM_OT_translate={"value": (0, 0, 0)})
                # TODO: copy mesh? This was taken from pull request #102
                # ob = blenderUtils.createPrimitive(collname, 'cylinder', (1,1,1),
                #                                   defs.layerTypes['collision'], materialname, center,
                #                                   rotation_euler)
                # ob.data = vis.data

            # set properties of new collision object
            ob.phobostype = 'collision'
            ob['geometry/type'] = self.property_colltype
            collisions.append(ob)

            # make collision object relative if visual object has a parent
            if vis.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                # CHECK test whether mesh option does work
                # this was taken from pull request #102
                # try:
                #     bpy.ops.object.transform_apply(scale=True)
                # except RuntimeError:
                #     log("Cannot apply scale. Mesh " + ob.data.name +
                #         " is shared between several objects.", "WARNING",
                #         "CreateCollisionObjects")
                vis.parent.select = True
                context.scene.objects.active = vis.parent
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                # TODO delete these lines?
                # ob.parent_type = vis.parent_type
                # ob.parent_bone = vis.parent_bone

            # select created collision objects
            sUtils.selectObjects(collisions)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


class SetCollisionGroupOperator(Operator):
    """Set the collision groups of the selected collision object(s)"""
    bl_idname = "phobos.set_collision_group"
    bl_label = "Set Collision Group(s)"
    bl_options = {'REGISTER', 'UNDO'}

    groups = BoolVectorProperty(
        name='Collision Groups',
        size=20,
        subtype='LAYER',
        default=(False,) * 20,
        description='Collision groups')

    def invoke(self, context, event):
        try:
            self.groups = context.active_object.rigid_body.collision_groups
        # create rigid body settings if not existent in active object
        except AttributeError:
            obj = context.active_object
            bpy.ops.rigidbody.object_add(type='ACTIVE')
            obj.rigid_body.kinematic = True
            obj.rigid_body.collision_groups = self.groups
        return self.execute(context)

    def execute(self, context):
        objs = filter(lambda e: "phobostype" in e and e.phobostype == "collision", context.selected_objects)
        active_object = context.active_object

        # try assigning the collision groups to each selected collision object
        for obj in objs:
            try:
                obj.rigid_body.collision_groups = self.groups
            # initialize rigid body settings if necessary
            except AttributeError:
                context.scene.objects.active = obj
                bpy.ops.rigidbody.object_add(type='ACTIVE')
                obj.rigid_body.kinematic = True
                obj.rigid_body.collision_groups = self.groups
        context.scene.objects.active = active_object
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.phobostype == 'collision' and ob.mode == 'OBJECT'


class DefineJointConstraintsOperator(Operator):
    """Add bone constraints to the joint (link)"""
    bl_idname = "phobos.define_joint_constraints"
    bl_label = "Define Joint(s)"
    bl_options = {'REGISTER', 'UNDO'}

    passive = BoolProperty(
        name='Passive',
        default=False,
        description='Make the joint passive (no actuation)'
    )

    useRadian = BoolProperty(
        name='Use Radian',
        default=True,
        description='Use degrees or rad for joints'
    )

    joint_type = EnumProperty(
        name='Joint Type',
        default='revolute',
        description="Type of the joint",
        items=defs.jointtypes)

    lower = FloatProperty(
        name="Lower",
        default=0.0,
        description="Lower constraint of the joint")

    upper = FloatProperty(
        name="Upper",
        default=0.0,
        description="Upper constraint of the joint")

    maxeffort = FloatProperty(
        name="Max Effort (N or Nm)",
        default=0.0,
        description="Maximum effort of the joint")

    maxvelocity = FloatProperty(
        name="Max Velocity (m/s or rad/s)",
        default=0.0,
        description="Maximum velocity of the joint. If you uncheck radian, you can enter °/sec here")

    spring = FloatProperty(
        name="Spring Constant",
        default=0.0,
        description="Spring constant of the joint")

    damping = FloatProperty(
        name="Damping Constant",
        default=0.0,
        description="Damping constant of the joint")

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "joint_type", text="joint_type")

        # enable/disable optional parameters
        if not self.joint_type == 'fixed':
            layout.prop(self, "passive", text="makes the joint passive (no actuation)")
            layout.prop(self, "useRadian", text="use radian")
            if self.joint_type != 'fixed':
                layout.prop(self, "maxeffort",
                            text="max effort [" + ('Nm]' if self.joint_type in ['revolute', 'continuous'] else 'N]'))
                if self.joint_type in ['revolute', 'continuous']:
                    layout.prop(self, "maxvelocity", text="max velocity [" + ("rad/s]" if self.useRadian else "°/s]"))
                else:
                    layout.prop(self, "maxvelocity", text="max velocity [m/s]")
            if self.joint_type in ('revolute', 'prismatic'):
                layout.prop(self, "lower", text="lower [rad]" if self.useRadian else "lower [°]")
                layout.prop(self, "upper", text="upper [rad]" if self.useRadian else "upper [°]")
                layout.prop(self, "spring", text="spring constant [N/m]")
                layout.prop(self, "damping", text="damping constant")

    def invoke(self, context, event):
        aObject = context.active_object
        if 'joint/type' not in aObject and 'motor/type' in aObject:
            self.maxvelocity = aObject['motor/maxSpeed']
            self.maxeffort = aObject['motor/maxEffort']
        return self.execute(context)

    def execute(self, context):
        log('Defining joint constraints for joint: ', 'INFO')
        lower = 0
        upper = 0

        # lower and upper limits
        if self.joint_type in ('revolute', 'prismatic'):
            if not self.useRadian:
                lower = math.radians(self.lower)
                upper = math.radians(self.upper)
            else:
                lower = self.lower
                upper = self.upper

        # velocity calculation
        if not self.useRadian:
            velocity = self.maxvelocity * ((2 * math.pi) / 360)  # from °/s to rad/s
        else:
            velocity = self.maxvelocity

        # set properties for each joint
        for joint in (obj for obj in context.selected_objects if obj.phobostype == 'link'):
            context.scene.objects.active = joint
            joints.setJointConstraints(joint, self.joint_type, lower, upper, self.spring, self.damping)

            # TODO is this still needed? Or better move it to the utility function
            if self.joint_type != 'fixed':
                joint['joint/maxeffort'] = self.maxeffort
                joint['joint/maxvelocity'] = velocity
            else:
                if "joint/maxeffort" in joint: del joint["joint/maxeffort"]
                if "joint/maxvelocity" in joint: del joint["joint/maxvelocity"]
            if self.passive:
                joint['joint/passive'] = "$true"
            else:
                # TODO show up in text edit which joints are to change?
                log("Please add motor to active joint in " + joint.name, "INFO")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        # due to invoke the active object needs to be a link
        return ob is not None and ob.phobostype == 'link' and ob.mode == 'OBJECT'


class AddMotorOperator(Operator):
    """Attach motor values to selected joints"""
    bl_idname = "phobos.add_motor"
    bl_label = "Add/Edit Motor"
    bl_options = {'REGISTER', 'UNDO'}

    P = FloatProperty(
        name="P",
        default=1.0,
        description="P value")

    I = FloatProperty(
        name="I",
        default=0.0,
        description="I value")

    D = FloatProperty(
        name="D",
        default=0.0,
        description="D value")

    addcontrollerparameters = BoolProperty(
        name="add controller parameters",
        default=False,
        description="whether or not to add PID control values"
    )

    vmax = FloatProperty(
        name="Maximum Velocity [m/s] or [rad/s]",
        default=1.0,
        description="Maximum turning velocity of the motor")

    taumax = FloatProperty(
        name="Maximum Torque [Nm]",
        default=1.0,
        description="Maximum torque a motor can apply")

    motortype = EnumProperty(
        name='Motor Type',
        default='generic_dc',
        description="Type of the motor",
        items=tuple([(t,) * 3 for t in defs.definitions['motors']])
        )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "motortype")
        layout.prop(self, "addcontrollerparameters")
        if not self.motortype == 'none':
            layout.prop(self, "taumax", text="maximum torque [Nm]")
            layout.prop(self, "vmax", text="maximum velocity [m/s] or [rad/s]")
            if self.addcontrollerparameters:
                layout.prop(self, "P", text="P")
                layout.prop(self, "I", text="I")
                layout.prop(self, "D", text="D")

    def invoke(self, context, event):
        aObject = context.active_object
        if 'motor/type' not in aObject and 'joint/type' in aObject and aObject['joint/type'] != 'fixed':
            self.taumax = aObject['joint/maxeffort']
            self.vmax = aObject['joint/maxvelocity']
        return self.execute(context)

    def execute(self, context):
        objs = (obj for obj in context.selected_objects if obj.phobostype == "link")
        for joint in objs:
            # add motor properties
            if not self.motortype == 'none':
                if self.addcontrollerparameters:
                    joint['motor/p'] = self.P
                    joint['motor/i'] = self.I
                    joint['motor/d'] = self.D
                joint['motor/maxSpeed'] = self.vmax
                joint['motor/maxEffort'] = self.taumax
                joint['motor/type'] = self.motortype
            # delete motor properties for none type
            else:
                for key in joint.keys():
                    if key.startswith('motor/'):
                        del joint[key]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        # due to invoke the active object needs to be a link
        return ob is not None and ob.phobostype == 'link' and ob.mode == 'OBJECT'


class CreateLinksOperator(Operator):
    """Create link(s), optionally based on existing objects"""
    bl_idname = "phobos.create_links"
    bl_label = "Create Link(s)"
    bl_options = {'REGISTER', 'UNDO'}

    location = EnumProperty(
        items=(('3D cursor',) * 3,
               ('selected objects',) * 3),
        default='3D cursor',
        name='Location',
        description='Where to create new link(s)?'
    )

    size = FloatProperty(
        name="Visual Size",
        default=1.0,
        description="Size of the created link"
    )

    parent_link = BoolProperty(
        name="Parent Link",
        default=False,
        description="Parent link to object's parents"
    )

    parent_objects = BoolProperty(
        name='Parent Objects',
        default=False,
        description='Parent children of object to new link'
    )

    nameformat = StringProperty(
        name="Name Format",
        description="Provide a string containing {0} {1} etc. to reuse parts of objects' names.",
        default=''
    )

    linkname = StringProperty(
        name="Link Name",
        description="A name for a single newly created link.",
        default='new_link'
    )

    def execute(self, context):
        if self.location == '3D cursor':
            modellinks.createLink({'name': self.linkname, 'scale': self.size})
        else:
            for obj in context.selected_objects:
                modellinks.deriveLinkfromObject(obj, scale=self.size, parent_link=self.parent_link,
                                           parent_objects=self.parent_objects,
                                           nameformat=self.nameformat)
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "location")
        layout.prop(self, "size")
        if self.location == '3D cursor':
            layout.prop(self, 'linkname')
        else:
            layout.prop(self, "nameformat")
            layout.prop(self, "parent_link")
            layout.prop(self, "parent_objects")

# TODO write more info in documentation once method works (where to add sensor
# etc.)

# TODO where can we put this...?
class DynamicProperty(bpy.types.PropertyGroup):
    """A support class to handle dynamic properties in a temporary operator."""
    name = bpy.props.StringProperty()
    intProp = bpy.props.IntProperty()
    boolProp = bpy.props.BoolProperty()
    stringProp = bpy.props.StringProperty()
    floatProp = bpy.props.FloatProperty()


def addSensorFromYaml(category, name):
    """This registers a temporary sensor Operator.
    The data for the properties is provided by the parsed yaml files of the
    sensors. The operator then adds a sensor object and writes the information
    to custom properties.
    The name is changed to fit the blender format of name1_name2_name3.
    """
    # unregister other temporary operators first
    # TODO use a proper if instead of try except
    try:
        bpy.utils.unregister_class(TempSensorOperator)
    except UnboundLocalError:
        pass

    blender_name = name.replace(' ', '_').lower()
    operatorBlenderId = 'phobos.add_sensor_' + blender_name

    # create the temporary operator class
    class TempSensorOperator(bpy.types.Operator):
        """Temporary operator to add a {0} sensor.""".format(name)
        bl_idname = operatorBlenderId
        bl_label = 'Add ' + name
        bl_description = 'Add a {0} sensor from the {1} category.'.format(
            name, category)
        bl_options = {'REGISTER'}

        # this contains all the single entries of the dictionary after invoking
        sensor_data = bpy.props.CollectionProperty(type=DynamicProperty)
        categ = category
        sensorName = name
        addLink = BoolProperty(default=False)

        def draw(self, context):
            layout = self.layout

            # expose the parameters as the right Property
            for i in range(len(self.sensor_data)):
                # use the dynamic props name in the GUI, but without the type id
                name = self.sensor_data[i].name[2:].replace('_', ' ')
                if self.sensor_data[i].name[0] == 'i':
                    layout.prop(self.sensor_data[i], 'intProp', text=name)
                elif self.sensor_data[i].name[0] == 'b':
                    layout.prop(self.sensor_data[i], 'boolProp', text=name)
                elif self.sensor_data[i].name[0] == 's':
                    layout.prop(self.sensor_data[i], 'stringProp', text=name)
                elif self.sensor_data[i].name[0] == 'f':
                    layout.prop(self.sensor_data[i], 'floatProp', text=name)

        def invoke(self, context, event):
            # load the sensor definitions of the current sensor
            data = defs.definitions['sensors'][self.categ][self.sensorName]

            hidden_props = ['general']
            # identify the property type for all the stuff in the definition
            for propname in data.keys():
                if propname in hidden_props:
                    continue
                item = self.sensor_data.add()
                prefix = ''
                if isinstance(data[propname], int):
                    item.intProp = data[propname]
                    prefix = 'i'
                elif isinstance(data[propname], str):
                    import re

                    # make sure eval is called only with true or false
                    if re.match('true|false', data[propname][1:], re.IGNORECASE):
                        booleanString = data[propname][1:]
                        booleanString = booleanString[0].upper() + booleanString[1:].lower()
                        item.boolProp = eval(booleanString)
                        prefix = 'b'
                    else:
                        item.stringProp = data[propname]
                        prefix = 's'
                elif isinstance(data[propname], float):
                    item.floatProp = data[propname]
                    prefix = 'f'
                # TODO what about lists?
                item.name = prefix + '_' + propname

            # open up the operator GUI
            return context.window_manager.invoke_props_dialog(self)

        def execute(self, context):
            sensordefs = defs.definitions['sensors'][self.categ][self.sensorName]
            # create a dictionary holding the sensor definition
            sensor_dict = {'name': self.sensorName,
                           'category': self.categ,
                           'material': 'phobos_sensor',
                           'props': {}
            }
            original_obj = context.active_object

            newlink = None
            if self.addLink:
                newlink = links.createLink({
                    'scale': 1.,
                    'name': 'link_' + self.sensorName,
                    'matrix': original_obj.matrix_world})
                joints.setJointConstraints(newlink, 'fixed', 0., 0., 0., 0.)

            # we don't need to check the parentlink, as the calling operator
            # does make sure it exists (or a new link is created instead)
            if 'phobostype' in original_obj and original_obj.phobostype != 'link':
                parentlink = sUtils.getEffectiveParent(
                    original_obj, ignore_selection=True)
            else:
                parentlink = original_obj

            # add the general settings for this sensor
            general_settings = sensordefs['general']
            for gensetting in general_settings.keys():
                sensor_dict[gensetting] = general_settings[gensetting]

            # store collected sensor data properties in the props dictionary
            for i in range(len(self.sensor_data)):
                if self.sensor_data[i].name[0] == 'b':
                    store = '$' + str(bool(self.sensor_data[i]['boolProp'])).lower()
                elif self.sensor_data[i].name[0] == 'i':
                    store = self.sensor_data[i]['intProp']
                elif self.sensor_data[i].name[0] == 's':
                    store = self.sensor_data[i]['stringProp']
                elif self.sensor_data[i].name[0] == 'f':
                    store = self.sensor_data[i]['floatProp']

                sensor_dict['props'][self.sensor_data[i].name[2:]] = store

            # the parent object will be either a new or an existing link
            if newlink:
                parent_obj = newlink
            else:
                parent_obj = parentlink

            pos_matrix = original_obj.matrix_world
            sensor_obj = sensors.createSensor(sensor_dict, parent_obj, pos_matrix)

            # parent to the added link
            if newlink:
                # parent newlink to parent link if there is any
                if parentlink:
                    sUtils.selectObjects([newlink, parentlink], clear=True, active=1)
                    bpy.ops.object.parent_set(type='BONE_RELATIVE')

            # parent sensor to its superior link
            sUtils.selectObjects([sensor_obj, parent_obj], clear=True, active=1)
            bpy.ops.object.parent_set(type='BONE_RELATIVE')

            # select the newly added objects
            if newlink:
                sUtils.selectObjects([sensor_obj, newlink], clear=True, active=0)
            else:
                sUtils.selectObjects([sensor_obj], clear=True, active=0)

            return {'FINISHED'}

    # register the temporary operator and return its blender id
    bpy.utils.register_class(TempSensorOperator)
    return operatorBlenderId


class AddSensorOperator(Operator):
    """Add a sensor at the position of the selected object.
    It is possible to create a new link for the sensor on the fly. Otherwise,
    the next link in the hierarchy will be used to parent the sensor to."""
    bl_idname = "phobos.add_sensor"
    bl_label = "Add Sensor"
    bl_options = {'REGISTER', 'UNDO'}

    def sensorlist(self, context):
        items = []
        for sensor in defs.definitions['sensors'][self.categ]:
            items.append((sensor,) * 3)
        return items

    def categorylist(self, context):
        '''Create an enum for the sensor categories. For phobos preset categories,
        the phobosIcon is added to the enum.
        '''
        from phobos.phobosgui import prev_collections

        phobosIcon = prev_collections["phobos"]["phobosIcon"].icon_id
        categories = [t for t in defs.definitions['sensors']]

        icon = ''
        items = []
        i = 0
        for categ in categories:
            # assign an icon to the phobos preset categories
            if categ == 'scanning':
                icon = 'OUTLINER_OB_FORCE_FIELD'
            elif categ == 'camera':
                icon = 'CAMERA_DATA'
            elif categ == 'internal':
                icon = 'PHYSICS'
            elif categ == 'environmental':
                icon = 'WORLD'
            elif categ == 'communication':
                icon = 'SPEAKER'
            else:
                icon = 'LAYER_USED'

            items.append((categ, categ, categ, icon, i))
            i += 1

        return items

    categ = EnumProperty(
        items=categorylist,
        description='The sensor category'
    )
    sensor = EnumProperty(
        items=sensorlist,
        description='The sensor type'
    )
    addLink = BoolProperty(
        name="Add link",
        default=True,
        description="Add additional link as sensor mounting"
    )
    sensorName = StringProperty(
        name="Sensor name",
        default='new_sensor',
        description="Name of the sensor"
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'sensorName')
        layout.separator()
        layout.prop(self, 'categ', text='Sensor category')
        layout.prop(self, 'sensor', text='Sensor type')
        layout.prop(self, 'addLink')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def check(self, context):
        return True

    @classmethod
    def poll(cls, context):
        return context.active_object

    def execute(self, context):
        # make sure a link is selected or created
        if not self.addLink and not sUtils.getEffectiveParent(
                context.active_object, ignore_selection=True):
            log('You have no link to add the sensor to. Select one ' +
                    'or create it with the operator.', 'INFO')
            return {'CANCELLED'}

        # match the operator to avoid dangers of eval
        import re
        operatorName = addSensorFromYaml(self.categ, self.sensor)
        operatorPattern = re.compile('[[a-z][a-zA-Z]*\.]*[a-z][a-zA-Z]*')

        # run the operator and pass on add link (to allow undo both new link and sensor)
        if operatorPattern.match(operatorName):
            eval('bpy.ops.' + operatorName + "('INVOKE_DEFAULT', " +
                 "addLink=" + str(self.addLink) +
                 ")")
        else:
            log('This sensor name is not following the naming convention: ' +
                operatorName + '. It can not be converted into an operator.',
                'ERROR')
        return {'FINISHED'}


def getControllerParameters(name):
    """Returns the controller parameters for the controller type with the provided
    name.

    Args:
      name(str): the name of the controller type.

    Returns:

    """
    try:
        return defs.definitions['controllers'][name]['parameters'].keys()
    except:
        return []


def getDefaultControllerParameters(scene, context):
    """Returns the default controller parameters for the controller of the active
    object.

    Args:
      scene:
      context:

    Returns:

    """
    try:
        name = bpy.context.active_object['motor/controller']
        return defs.definitions['controllers'][name]['parameters'].values()
    except:
        return None


class CreateMimicJointOperator(Operator):
    """Make a number of joints follow a specified joint"""
    bl_idname = "phobos.create_mimic_joint"
    bl_label = "Mimic Joint"
    bl_options = {'REGISTER', 'UNDO'}

    multiplier = FloatProperty(
        name="Multiplier",
        default=1.0,
        description="Multiplier for joint mimicry")

    offset = FloatProperty(
        name="Offset",
        default=0.0,
        description="Offset for joint mimicry")

    mimicjoint = BoolProperty(
        name="Mimic Joint",
        default=True,
        description="Create joint mimicry")

    mimicmotor = BoolProperty(
        name="Mimic Motor",
        default=False,
        description="Create motor mimicry")

    def execute(self, context):
        masterjoint = context.active_object
        objs = filter(lambda e: "phobostype" in e and e.phobostype == "link", context.selected_objects)

        # apply mimicking for all selected joints
        for obj in objs:
            if obj.name != masterjoint.name:
                if self.mimicjoint:
                    obj["joint/mimic_joint"] = nUtils.getObjectName(masterjoint, 'joint')
                    obj["joint/mimic_multiplier"] = self.multiplier
                    obj["joint/mimic_offset"] = self.offset
                if self.mimicmotor:
                    obj["motor/mimic_motor"] = nUtils.getObjectName(masterjoint, 'motor')
                    obj["motor/mimic_multiplier"] = self.multiplier
                    obj["motor/mimic_offset"] = self.offset
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        objs = list(filter(lambda e: "phobostype" in e and e.phobostype ==
                           "link", context.selected_objects))
        return (ob is not None and ob.phobostype == 'link' and len(objs) > 1)


class AddHeightmapOperator(Operator):
    """Add a heightmap object to the 3D-Cursors location"""
    bl_idname = "phobos.add_heightmap"
    bl_label = "Create heightmap"
    bl_options = {'REGISTER', 'UNDO'}

    name = StringProperty(
        name="Name",
        description="The name of the new heightmap object",
        default="heightmap")

    cutNo = IntProperty(
        name="Number of cuts",
        description="Number of cuts for subdivision",
        default=100)

    strength = FloatProperty(
        name="Displacement strength",
        description="Strength of the displacement effect",
        default=0.1)

    subsurf = BoolProperty(
        name="Use subsurf",
        description="Use subsurf modifier to smoothen surface",
        default=False)

    subsurflvl = IntProperty(
        name="Subsurf subdivisions",
        description="Number of divisions for subsurf smoothing",
        default=2)

    filepath = StringProperty(subtype="FILE_PATH")

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'name')
        layout.prop(self, 'cutNo')
        layout.prop(self, 'strength')
        layout.prop(self, 'subsurf')
        if self.subsurf:
            layout.prop(self, 'subsurflvl')

    def execute(self, context):
        if os.path.basename(self.filepath) not in bpy.data.images:
            try:
                img = bpy.data.images.load(self.filepath)
            except RuntimeError:
                log("Cannot load image from file! Aborting.", "ERROR")
                return {"CANCELLED"}
        else:
            log("Image already imported. Using cached version.", "INFO")
            img = bpy.data.images[os.path.basename(self.filepath)]

        # Create texture from image
        h_tex = bpy.data.textures.new(self.name, type='IMAGE')
        h_tex.image = img

        # Add plane as single object (phobostype visual)
        if context.scene.objects.active:
            bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.mesh.primitive_plane_add(view_align=False, enter_editmode=False)
        plane = context.active_object
        plane['phobostype'] = 'visual'

        # subdivide plane
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.subdivide(number_cuts=self.cutNo)
        bpy.ops.object.mode_set(mode='OBJECT')

        # create displacement
        plane.modifiers.new('displace_heightmap', 'DISPLACE')
        plane.modifiers['displace_heightmap'].texture = h_tex
        plane.modifiers['displace_heightmap'].strength = self.strength

        # create subsurf
        if self.subsurf:
            plane.modifiers.new('subsurf_heightmap', 'SUBSURF')
            plane.modifiers['subsurf_heightmap'].render_levels = self.subsurflvl

        # enable smooth shading
        bpy.ops.phobos.smoothen_surface()

        # Add root link for heightmap
        root = modellinks.deriveLinkfromObject(plane, scale=1.0, parenting=True,
                                          parentobjects=True)

        # set names and custom properties
        # FIXME: what about the namespaces? @HEIGHTMAP (14)
        plane.name = self.name + '_visual::heightmap'
        root.name = self.name + "::heightmap"
        root['entity/type'] = 'heightmap'
        root['entity/name'] = self.name
        root['image'] = os.path.relpath(os.path.basename(self.filepath), bpy.data.filepath)
        root['joint/type'] = 'fixed'

        # select the plane object for further adjustments
        context.scene.objects.active = plane

        # FIXME this GUI "hack" does not work, as the buttons context enum is not updated while the operator is running @HEIGHTMAP (30)
        # current_screen = bpy.context.screen.name
        # screen = bpy.data.screens[current_screen]
        # for area in screen.areas:
        #     if area.type == 'PROPERTIES':
        #         area.tag_redraw()
        #         area.spaces[0].context = 'MODIFIER'
        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class AddAnnotationsOperator(bpy.types.Operator):
    """Add annotations defined in a YAML file"""
    bl_idname = "phobos.add_annotations"
    bl_label = "Add Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    def getAnnotationTypes(self, context):
        return [(category,) * 3 for category in sorted(defs.definitions.keys())]

    def getDeviceTypes(self, context):
        return [(category,) * 3 for category in sorted(defs.definitions[self.annotationtype].keys())]

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    annotationtype = EnumProperty(
        items=getAnnotationTypes,
        name="Annotation Type",
        description="Annotation Types")

    devicetype = EnumProperty(
        items=getDeviceTypes,
        name="Device Type",
        description="Device Types")

    @classmethod
    def poll(cls, context):
        return context is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        l = self.layout
        l.prop(self, 'filepath')
        if self.filepath == '':
            l.prop(self, 'annotationtype')
            l.prop(self, 'devicetype')
            b = self.layout.box()
            try:
                for key, value in defs.definitions[self.annotationtype][self.devicetype].items():
                    b.label(text=key+': '+str(value))
            except KeyError:
                pass  # no valid key selected yet

    def execute(self, context):
        if self.filepath != '':
            try:
                with open(self.filepath, 'r') as annotationfile:
                    annotations = yaml.load(annotationfile.read())
                for category in annotations:
                    for key, value in annotations[category].items():
                        context.active_object[category+'/'+key] = value
            except FileNotFoundError:
                log("Annotation file seems to be invalid.", "ERROR")
        else:
            for key, value in defs.definitions[self.annotationtype][self.devicetype].items():
                for obj in context.selected_objects:
                    obj[self.devicetype+'/'+key] = value
        return {'FINISHED'}


class AddSubmodel(Operator):
    """Add a submodel instance to the scene"""
    bl_idname = "phobos.add_submodel"
    bl_label = "Add submodel"
    bl_options = {'REGISTER', 'UNDO'}

    def submodelnames(self, context):
        """Returns a list of submodels of the chosen type for use as enum

        Args:
          context:

        Returns:

        """
        submodellist = [a.name for a in bpy.data.groups
                        if 'submodeltype' in a
                        and a['submodeltype'] == self.submodeltype]
        return [(a,
                 a.split(':')[1],
                 a.split(':')[1].split('/')[0] +
                 ' version ' + a.split(':')[1].split('/')[1] + ' submodel')
                for a in submodellist]

    def submodeltypes(self, context):
        """Returns a list of submodel types in the scene for use as enum

        Args:
          context:

        Returns:

        """
        submodellist = [a['submodeltype'] for a in bpy.data.groups
                        if 'submodeltype' in a]
        submodellist = set(submodellist)
        return [(a, a, a + ' submodel type') for a in submodellist]

    submodeltype = EnumProperty(
        name="Submodel type",
        description="Type of the submodel",
        items=submodeltypes
    )

    submodelname = EnumProperty(
        name="Submodel name",
        description="Name of the submodel",
        items=submodelnames
    )

    instancename = StringProperty(
        name="Instance name",
        default=''
    )

    def check(self, context):
        return True

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'submodeltype')
        layout.prop(self, 'submodelname')
        layout.prop(self, 'instancename')

    def invoke(self, context, event):
        """Start off the instance numbering based on Blender objects and show
        a property dialog

        Args:
          context:
          event:

        Returns:

        """
        self.instancename = self.submodelname.split(':')[1].split('/')[0]
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    @classmethod
    def poll(cls, context):
        """Hide the operator when no submodels are defined

        Args:
          context:

        Returns:

        """
        for group in bpy.data.groups:
            if 'submodeltype' in group:
                return True
        return False


    def execute(self, context):
        """create an instance of the submodel

        Args:
          context:

        Returns:

        """
        i = 1
        while self.instancename + '_{0:03d}'.format(i) in bpy.data.objects:
            i += 1
        objectname = self.instancename + '_{0:03d}'.format(i)
        eUtils.instantiateSubmodel(self.submodelname, objectname)
        return {'FINISHED'}


class DefineSubmodel(Operator):
    """Define a new submodel from objects"""
    bl_idname = "phobos.define_submodel"
    bl_label = "Define Submodel"
    bl_options = {'REGISTER', 'UNDO'}

    submodelname = StringProperty(
        name="Submodel name",
        description="Name of the submodel",
        default='newsubmodel'
    )

    version = StringProperty(
        name="Version name",
        description="Name of the submodel version",
        default='1.0'
    )

    submodeltype = EnumProperty(
        items=tuple([(sub,) * 3 for sub in defs.definitions['submodeltypes']]),
        name="Submodel type",
        default="mechanism",
        description="The type for the new submodel"
    )

    def invoke(self, context, event):
        """Show a property dialog

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        """Create a submodel based on selected objects

        Args:
          context:

        Returns:

        """
        eUtils.defineSubmodel(self.submodelname,
                              self.submodeltype,
                              self.version)
        return {'FINISHED'}


class AssignSubmechanism(Operator):
    """Assign a submechanism to the model"""
    bl_idname = "phobos.assign_submechanism"
    bl_label = "Assign Submechanism"
    bl_options = {'REGISTER', 'UNDO'}

    #mechanism_type = EnumProperty(
    #    name="Submechanism type",
    #    items=bUtils.compileEnumPropertyList(defs.definitions['submechanisms'].keys()),
    #    )
            #maybe add size in brackets? lambda_mechanism(3) / [3] lambda_mechanism

    #mechanism_category = EnumProperty(
    #    name="Submechanism category",
    #    items=bUtils.compileEnumPropertyList(defs.definitions['submechanisms'].keys()),
    #    )

    linear_chain = BoolProperty(name='Serial Chain', default=False)

    mechanism_name = StringProperty(name='Name')

    joints = []

    def compileSubmechanismTreeEnum(self, context):
        return bUtils.compileEnumPropertyList(
            defs.definitions['submechanisms'][context.window_manager.mechanismpreview]['joints']['spanningtree'])

    def isLinearChain(self, jointlist):
        leafs = [joint for joint in jointlist if all([c not in jointlist for c in joint.children])]
        chain = []
        if len(leafs) == 1:
            count = 1
            child = leafs[0]
            chain.append(child)
            while child.parent in jointlist:
                child = child.parent
                chain.append(child)
                count += 1
            return tuple(reversed(chain))
        else:
            return jointlist

    jointtype0 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype1 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype2 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype3 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype4 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype5 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype6 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype7 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype8 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype9 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype10 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype11 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype12 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype13 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype14 = EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype15 = EnumProperty(items=compileSubmechanismTreeEnum)


    @classmethod
    def poll(cls, context):
        return (len(bpy.context.selected_objects) > 0 and
                any((a.phobostype == 'link' for a in bpy.context.selected_objects)))

    def draw(self, context):
        wm = context.window_manager
        layout = self.layout
        layout.label('Selection contains {0} joints.'.format(len(self.joints)))
        layout.prop(self, 'linear_chain')
        layout.prop(self, 'mechanism_name')
        if not self.linear_chain:
            layout.template_icon_view(wm, 'mechanismpreview', show_labels=True, scale=5.0)
            layout.prop(wm, 'mechanismpreview')
            size = len(defs.definitions['submechanisms'][wm.mechanismpreview]['joints']['spanningtree'])
            if size == len(self.joints):
                glayout = layout.split()
                c1 = glayout.column(align=True)
                c2 = glayout.column(align=True)
                for i in range(size):
                    c1.label(nUtils.getObjectName(self.joints[i], 'joint') + ':')
                    c2.prop(self, "jointtype" + str(i), text='')
            else:
                layout.label('Please choose a valid type for selected joints.')

    def execute(self, context):
        self.joints = self.isLinearChain([obj for obj in bpy.context.selected_objects
                                          if obj.phobostype == 'link'])
        # prepare data used in both cases
        roots = [link for link in self.joints if link.parent not in self.joints]
        if len(roots) != 1:
            log("Selected joints are not all connected.", 'WARNING')
            if self.linear_chain:
                return {'CANCELLED'}
        if self.mechanism_name:
            if self.linear_chain:
                root = roots[0]
                root['submechanism/type'] = '{0}R'.format(len(self.joints))
                root['submechanism/spanningtree'] = list(self.joints)
                root['submechanism/active'] = list(self.joints)
                root['submechanism/independent'] = list(self.joints)
                for i in range(len(self.joints)):
                    self.joints[i]['submechanism/jointname'] = str(i+1)
            else:
                root, freeloader_joints = eUtils.getNearestCommonParent(self.joints)
                mechanismdata = defs.definitions['submechanisms'][context.window_manager.mechanismpreview]
                size = len(mechanismdata['joints']['spanningtree'])
                if len(self.joints) == size:
                    jointmap = {getattr(self, 'jointtype'+str(i)): self.joints[i] for i in range(len(self.joints))}
                    # assign attributes
                    try:
                        for i in range(len(self.joints)):
                            self.joints[i]['submechanism/jointname'] = getattr(self, 'jointtype'+str(i))
                        root['submechanism/type'] = mechanismdata['type']
                        root['submechanism/subtype'] = context.window_manager.mechanismpreview
                        root['submechanism/spanningtree'] = [jointmap[j] for j in mechanismdata['joints']['spanningtree']]
                        root['submechanism/active'] = [jointmap[j] for j in mechanismdata['joints']['active']]
                        root['submechanism/independent'] = [jointmap[j] for j in mechanismdata['joints']['independent']]
                        root['submechanism/root'] = root
                        root['submechanism/freeloader'] = freeloader_joints
                    except KeyError:
                        log("Incomplete joint definition.")
                else:
                    log('Number of joints not valid for selected submechanism type: ' +
                        context.window_manager.mechanismpreview, 'ERROR')
                    return {'FINISHED'}
            root['submechanism/name'] = self.mechanism_name
        else:
            log('Submechanism definition requires valid name.', 'WARNING')
        return {'FINISHED'}


class SelectSubmechanism(Operator):
    """Select all objects of a submechanism"""
    bl_idname = "phobos.select_submechanism"
    bl_label = "Select Submechanism"
    bl_options = {'REGISTER', 'UNDO'}

    def get_submechanism_roots(self, context):
        return bUtils.compileEnumPropertyList([r['submechanism/name']
                                               for r in sUtils.getSubmechanismRoots()])

    submechanism = EnumProperty(
        name="Submechanism",
        description="submechanism which to select",
        items=get_submechanism_roots
    )

    def execute(self, context):
        root = sUtils.getObjectByProperty('submechanism/name', self.submechanism)
        jointlist = root['submechanism/spanningtree']
        sUtils.selectObjects([root] + jointlist, clear=True, active=0)
        return {'FINISHED'}


class ToggleInterfaces(Operator):
    """Toggle interfaces of a submodel"""
    bl_idname = "phobos.toggle_interfaces"
    bl_label = "Toggle Interfaces"
    bl_options = {'REGISTER', 'UNDO'}

    mode = EnumProperty(
        name="Toggle mode",
        description="The mode in which to display the interfaces",
        items=(('toggle',) * 3, ('activate',) * 3, ('deactivate',) * 3)
    )

    def execute(self, context):
        eUtils.toggleInterfaces(None, self.mode)
        return {'FINISHED'}


class ConnectInterfacesOperator(Operator):
    """Connects submodels at interfaces"""
    bl_idname = "phobos.connect_interfaces"
    bl_label = "Connect Interfaces"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        """Hide operator if there are more than two objects are selected and the interfaces do not
        match."""
        try:
            # no interface selected
            if (context.active_object is None or len(context.selected_objects) != 2 or not
                    all([obj.phobostype == 'interface' for obj in context.selected_objects])):
                return False

            parentinterface = context.active_object
            childinterface = [a for a in context.selected_objects if a != context.active_object][0]
            # check for same interface type and directions
            if ((parentinterface['interface/type'] == childinterface['interface/type']) and
                (parentinterface['interface/direction'] != childinterface['interface/direction'])
                or (parentinterface['interface/direction'] == 'bidirectional' and
                    childinterface['interface/direction'] == 'bidirectional')):
                return True
            else:
                return False
        except (KeyError, IndexError):  # if relevant data or selection is incorrect
            return False

    def execute(self, context):
        pi = 0 if context.selected_objects[0] == context.active_object else 1
        ci = int(not pi)  #0 if pi == 1 else 1
        parentinterface = context.selected_objects[pi]
        childinterface = context.selected_objects[ci]
        eUtils.connectInterfaces(parentinterface, childinterface)
        return {'FINISHED'}


class DisconnectInterfaceOperator(Operator):
    """Disconnects submodels at interface"""
    bl_idname = "phobos.disconnect_interface"
    bl_label = "Disconnect Interface"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        """Hide operator if there is more than one object selected.
        Also, the selected object has to be a connected interface.
        """
        # no interface selected
        if (context.active_object is None or len(context.selected_objects) != 1 or not
                context.active_object.phobostype == 'interface'):
            return False

        # interface needs to be connect to another interface
        interface = bpy.context.active_object
        if (interface.parent and interface.parent.phobostype == 'interface'):
            return True
        elif (interface.children and any([obj.phobostype for obj in interface.children])):
            return True
        return False

    def execute(self, context):
        """Execute disconnection"""
        interface = context.active_object
        if (interface.parent and interface.parent.phobostype == 'interface'):
            log('Selected interface is child.', 'DEBUG')
            child = interface
            parent = interface.parent
        else:
            log('Selected interface is parent.', 'DEBUG')
            parent = interface
            for curchild in interface.children:
                if curchild.phobostype == 'interface':
                    child = curchild
                    break
            log('Selected ' + child.name + ' as child.', 'DEBUG')

        eUtils.disconnectInterfaces(parent, child)
        return {'FINISHED'}


class MergeLinks(Operator):
    """Merge links"""
    bl_idname = "phobos.merge_links"
    bl_label = "Merge Links"
    bl_options = {'REGISTER', 'UNDO'}

    movetotarget = BoolProperty(name='move to target', default=False)

    @classmethod
    def poll(cls, context):
        return (context.active_object is not None and
                context.active_object.phobostype == 'link'
                and len([obj for obj in context.selected_objects
                         if obj.phobostype == 'link' and obj != context.active_object]) > 0)

    def execute(self, context):
        mergelinks = [obj for obj in context.selected_objects
                      if obj.phobostype == 'link' and obj != context.active_object]
        eUtils.mergeLinks(mergelinks, targetlink=context.active_object,
                          movetotarget=self.movetotarget)
        return {'FINISHED'}


class SetModelRoot(Operator):
    """Set Model Root"""
    bl_idname = "phobos.set_model_root"
    bl_label = "Set Model Root"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.phobostype == 'link'

    def execute(self, context):
        eUtils.restructureKinematicTree(context.object)
        return {'FINISHED'}


def register():
    print("Registering operators.editing...")
    # TODO this seems not to be very convenient...
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    print("Unregistering operators.editing...")
    # TODO this seems not to be very convenient...
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)


# TODO remove or use? Give it a dev branch
# class SelectErrorOperator(Operator):
#     """Select an object with check errors"""
#     bl_idname = "phobos.select_error"
#     bl_label = "Select Erroneous Object"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     errorObj = EnumProperty(
#         name="Erroneous Objects",
#         items=defs.generateCheckMessages,
#         description="The objects containing errors")
#
#     def execute(self, context):
#         sUtils.selectByName(self.errorObj)
#         for message in vUtils.checkMessages[self.errorObj]:
#             log(message, 'INFO')
#
#         return {'FINISHED'}


class ValidateOperator(Operator):
    """Check the robot dictionary"""
    bl_idname = "phobos.validate"
    bl_label = "Validate"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        messages = {}
        root = sUtils.getRoot(context.selected_objects[0])
        model, objectlist = models.deriveModelDictionary(root)
        vUtils.check_dict(model, defs.definitions['model'], messages)
        vUtils.checkMessages = messages if len(list(messages.keys())) > 0 else {"NoObject": []}
        for entry in messages:
            log("Errors in object " + entry + ":", 'INFO')
            for error in messages[entry]:
                log(error, 'INFO')
        return {'FINISHED'}


class CalculateMassOperator(Operator):
    """Display mass of the selected objects in a pop-up window"""
    bl_idname = "phobos.calculate_mass"
    bl_label = "Calculate Mass"

    mass = FloatProperty(
        name='Mass',
        default=0.0,
        description="Calculated sum of masses")

    def invoke(self, context, event):
        self.mass = gUtils.calculateSum(context.selected_objects, 'inertial/mass')
        return context.window_manager.invoke_popup(self)

    def execute(self, context):
        log("The calculated mass is: " + str(self.mass), "INFO")
        return {'FINISHED'}

    def draw(self, context):
        self.layout.label("Sum of masses: " + str(self.mass))


class MeasureDistanceOperator(Operator):
    """Show distance between two selected objects in world coordinates"""
    bl_idname = "phobos.measure_distance"
    bl_label = "Measure Distance"
    bl_options = {'REGISTER'}

    distance = FloatProperty(
        name="Distance",
        default=0.0,
        subtype='DISTANCE',
        unit='LENGTH',
        precision=6,
        description="Distance between objects")

    distVector = FloatVectorProperty(
        name="Distance Vector",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="Distance vector between objects")

    def execute(self, context):
        self.distance, self.distVector = gUtils.distance(context.selected_objects)
        log("distance: " + str(self.distance) + ", " + str(self.distVector), "INFO")
        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        return len(context.selected_objects) == 2
