#!/usr/bin/python
# coding=utf-8

"""
.. module:: phobos.operators.editing
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators to manipulate blender objects

.. moduleauthor:: Kai von Szadowski, Ole Schwiegert

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
"""

import math
import os
import yaml
from datetime import datetime

import bpy
import mathutils
from bpy.types import Operator
from bpy.props import IntProperty, StringProperty, FloatProperty, BoolProperty, EnumProperty, FloatVectorProperty, \
    BoolVectorProperty

import phobos.defs as defs
import phobos.inertia as inertia
import phobos.utils.selection as sUtils
import phobos.utils.general as gUtils
import phobos.utils.blender as bUtils
import phobos.utils.naming as nUtils
import phobos.joints as joints
import phobos.sensors as sensors
import phobos.links as links
from phobos.logging import startLog, endLog, log


class SortObjectsToLayersOperator(Operator):
    """Sort all selected objects to their according layers"""
    bl_idname = "object.phobos_sort_objects_to_layers"
    bl_label = "Sort Objects to Layers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        objs = filter(lambda e: "phobostype" in e, context.selected_objects)
        for obj in objs:
            phobosType = obj.phobostype
            if phobosType != 'controller' and phobosType != 'undefined':
                layers = 20 * [False]
                layers[defs.layerTypes[phobosType]] = True
                obj.layers = layers
            if phobosType == 'undefined':
                log("The phobostype of the object '" + obj.name + "' is undefined", "INFO")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


class AddChainOperator(Operator):
    """Add a chain between two selected objects"""
    bl_idname = "object.phobos_add_chain"
    bl_label = "Add Chain"
    bl_options = {'REGISTER', 'UNDO'}

    chainname = StringProperty(
        name='Chain Name',
        default='new_chain',
        description='Name of the chain to be created')

    def execute(self, context):
        endobj = context.active_object
        for obj in context.selected_objects:
            if obj is not context.active_object:
                startobj = obj
                break
        if not 'startChain' in startobj:
            startobj['startChain'] = [self.chainname]
        else:
            namelist = startobj['startChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            startobj['startChain'] = namelist
        if not 'endChain' in endobj:
            endobj['endChain'] = [self.chainname]
        else:
            namelist = endobj['endChain']
            if self.chainname not in namelist:
                namelist.append(self.chainname)
            endobj['endChain'] = namelist
        return {'FINISHED'}


class SetMassOperator(Operator):
    """Set the mass of the selected object(s)"""
    bl_idname = "object.phobos_set_mass"
    bl_label = "Set Mass"
    bl_options = {'REGISTER', 'UNDO'}

    mass = FloatProperty(
        name='Mass',
        default=0.001,
        description='Mass (of active object) in kg')

    userbmass = BoolProperty(
        name='Use Rigid Body Mass',
        default=False,
        description='If true, mass entry from rigid body data is used')

    @classmethod
    def poll(cls, context):
        return context.active_object and len(list(filter(lambda e: "phobostype" in e and
            e.phobostype in ("visual", "collision", "inertial"), context.selected_objects))) >= 1

    def invoke(self, context, event):
        if 'mass' in context.active_object:
            self.mass = context.active_object['mass']
        return self.execute(context)

    def execute(self, context):
        startLog(self)
        objs = filter(lambda e: "phobostype" in e and e.phobostype in ("visual", "collision", "inertial"), context.selected_objects)
        for obj in objs:
            try:
                oldmass = obj['mass']
            except KeyError:
                oldmass = None
            if self.userbmass:
                try:
                    obj['mass'] = obj.rigid_body.mass
                except AttributeError:
                    obj['mass'] = 0.001
                    # print("### Error: object has no rigid body properties.")
                    log("The object '" + obj.name + "' has no rigid body properties. Set mass to 0.001", "ERROR")
            else:
                obj['mass'] = self.mass
            if obj['mass'] != oldmass:
                t = datetime.now()
                obj['masschanged'] = t.isoformat()
        endLog()
        return {'FINISHED'}


class SyncMassesOperator(Operator):
    """Synchronize masses among the selected object(s)"""
    bl_idname = "object.phobos_sync_masses"
    bl_label = "Synchronize Masses"
    bl_options = {'REGISTER', 'UNDO'}

    synctype = EnumProperty(
        items=(("vtc", "visual to collision", "visual to collision"),
               ("ctv", "collision to visual", "collision to visual"),
               ("lto", "latest to oldest", "latest to oldest")),
        name="synctype",
        default="vtc",
        description="Phobos object type")

    updateinertial = BoolProperty(
        name='robotupdate inertial',
        default=False,
        description='Update inertials'
    )

    def execute(self, context):
        sourcelist = []
        targetlist = []
        processed = set()
        links = [obj.name for obj in context.selected_objects if obj.phobostype == 'link']
        t = datetime.now()
        objdict = {obj.name: obj for obj in bpy.data.objects if obj.phobostype in ['visual', 'collision']
                   and obj.parent.name in links}
        # gather all name bases of objects for which both visual and collision are present
        for obj in objdict.keys():
            basename = obj.replace(objdict[obj].phobostype + '_', '')
            if 'visual_' + basename in objdict.keys() and 'collision_' + basename in objdict.keys():
                processed.add(basename)
        # fill source and target lists for syncing
        for basename in processed:
            if self.synctype == "vtc":
                sourcelist.append('visual_' + basename)
                targetlist.append('collision_' + basename)
            elif self.synctype == "ctv":
                targetlist.append('visual_' + basename)
                sourcelist.append('collision_' + basename)
            else:  # latest to oldest
                try:
                    tv = gUtils.datetimeFromIso(objdict['visual_' + basename]['masschanged'])
                    tc = gUtils.datetimeFromIso(objdict['collision_' + basename]['masschanged'])
                    if tc < tv:  # if collision information is older than visual information
                        sourcelist.append('visual_' + basename)
                        targetlist.append('collision_' + basename)
                    else:
                        targetlist.append('visual_' + basename)
                        sourcelist.append('collision_' + basename)
                except KeyError:
                    print(basename, "has insufficient data for time-based synchronisation of masses.")
        # sync the mass values
        for i in range(len(sourcelist)):
            try:
                objdict[targetlist[i]]['mass'] = objdict[sourcelist[i]]['mass']
            except KeyError:
                print("No mass information in object", targetlist[i])
            if self.synctype != "vtc" and self.synctype != "ctv":
                objdict[targetlist[i]]['masschanged'] = objdict[sourcelist[i]]['masschanged']

        for linkname in links:
            masssum = 0.0
            link = bpy.data.objects[linkname]
            viscols = inertia.getInertiaRelevantObjects(link)
            for obj in viscols:
                masssum += obj['mass']
            link['mass'] = masssum
            link['masschanged'] = t.isoformat()
            if self.updateinertial:
                inertia.createInertials(link)
        return {'FINISHED'}


class SetXRayOperator(Operator):
    """Show the selected/chosen objects via X-ray"""
    bl_idname = "object.phobos_set_xray"
    bl_label = "Set X-Ray View"
    bl_options = {'REGISTER', 'UNDO'}

    objects = EnumProperty(
        name="Objects",
        default='selected',
        items=(('all',) * 3, ('selected',) * 3, ('by name',) * 3) + defs.phobostypes,
        description="Show objects via x-ray")

    show = BoolProperty(
        name="Show",
        default=True,
        description="Set to")

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
        layout.prop(self, "show", text="enable X-Ray view" if self.show else "disable X-Ray view")
        if self.objects == 'by name':
            layout.prop(self, "Name Part")

    def execute(self, context):
        if self.objects == 'all':
            objlist = bpy.data.objects
        elif self.objects == 'selected':
            objlist = context.selected_objects
        elif self.objects == 'by name':
            objlist = [obj for obj in bpy.data.objects if obj.name.find(self.namepart) >= 0]
        else:
            objlist = [obj for obj in bpy.data.objects if obj.phobostype == self.objects]
        for obj in objlist:
            obj.show_x_ray = self.show
        return {'FINISHED'}


class SetPhobosType(Operator):
    """Edit phobostype of selected object(s)"""
    bl_idname = "object.phobos_set_phobostype"
    bl_label = "Set Phobostype"
    bl_options = {'REGISTER', 'UNDO'}

    phobostype = EnumProperty(
        items=defs.phobostypes,
        name="Phobostype",
        default="undefined",
        description="Phobostype")

    def execute(self, context):
        for obj in context.selected_objects:
            obj.phobostype = self.phobostype
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class BatchEditPropertyOperator(Operator):
    """Edit custom property of selected object(s)"""
    bl_idname = "object.phobos_batch_property"
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
        if value == '':
            for obj in context.selected_objects:
                if self.property_name in obj.keys():
                    del(obj[self.property_name])
        else:
            for obj in context.selected_objects:
                obj[self.property_name] = value
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


class CopyCustomProperties(Operator):
    """Copy custom properties of selected object(s)"""
    bl_idname = "object.phobos_copy_props"
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
    bl_idname = "object.phobos_rename_custom_property"
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
        startLog(self)
        objs = filter(lambda e: self.find in e, context.selected_objects)
        if self.replace != "":
            for obj in objs:
                if self.replace in obj and not self.overwrite:
                    log("Property '" + self.replace + "' already present in object '" + obj.name + "'", "ERROR")
                else:
                    obj[self.replace] = obj[self.find]
                    del obj[self.find]
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetGeometryType(Operator):
    """Edit geometry type of selected object(s)"""
    bl_idname = "object.phobos_set_geometry_type"
    bl_label = "Edit Geometry"
    bl_options = {'REGISTER', 'UNDO'}

    geomType = EnumProperty(
        items=defs.geometrytypes,
        name="Type",
        default="box",
        description="Phobos geometry type")

    def execute(self, context):
        startLog(self)
        objs = filter(lambda e: "phobostype" in e, context.selected_objects)
        for obj in objs:
            if obj.phobostype == 'collision' or obj.phobostype == 'visual':
                obj['geometry/type'] = self.geomType
            else:
                log("The object '" + obj.name + "' is no collision or visual")
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class EditInertia(Operator):
    """Edit inertia of selected object(s)"""
    bl_idname = "object.phobos_edit_inertia"
    bl_label = "Edit Inertia"
    bl_options = {'REGISTER', 'UNDO'}

    # inertiamatrix = FloatVectorProperty (
    # name = "inertia",
    # default = [0, 0, 0, 0, 0, 0, 0, 0, 0],
    # subtype = 'MATRIX',
    #        size = 9,
    #        description = "set inertia for a link")

    inertiavector = FloatVectorProperty(
        name="Inertia Vector",
        default=[0, 0, 0, 0, 0, 0],
        subtype='NONE',
        size=6,
        description="Set inertia for a link"
    )

    def invoke(self, context, event):
        if 'inertia' in context.active_object:
            self.inertiavector = mathutils.Vector(context.active_object['inertia'])
        return self.execute(context)

    def execute(self, context):
        # m = self.inertiamatrix
        # inertialist = []#[m[0], m[1], m[2], m[4], m[5], m[8]]
        # obj['inertia'] = ' '.join(inertialist)
        objs = filter(lambda e: "phobostype" in e and e.phobostype == "inertial", context.selected_objects)
        for obj in objs:
            obj['inertia'] = self.inertiavector  # ' '.join([str(i) for i in self.inertiavector])
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and ob.phobostype == 'inertial' and len(
            context.selected_objects) > 0


class SmoothenSurfaceOperator(Operator):
    """Smoothen surface of selected objects"""
    bl_idname = "object.phobos_smoothen_surface"
    bl_label = "Smoothen Surface"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269;
        objs = filter(lambda e: e.type == "MESH", context.selected_objects)
        if show_progress:
            wm = context.window_manager
            total = float(len(context.selected_objects))
            wm.progress_begin(0, total)
            i = 1
        for obj in objs:
            context.scene.objects.active = obj
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all()
            bpy.ops.mesh.normals_make_consistent()
            bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.shade_smooth()
            bpy.ops.object.modifier_add(type='EDGE_SPLIT')
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetOriginToCOMOperator(Operator):
    """Set origin to COM"""
    bl_idname = "object.phobos_set_origin_to_com"
    bl_label = "Set Origin to COM"
    bl_options = {'REGISTER', 'UNDO'}

    com_shift = FloatVectorProperty(
        name="CAD Origin Shift",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="Offset of distance between objects")

    cursor_location = FloatVectorProperty(
        name="CAD Origin",
        default=(0.0, 0.0, 0.0,),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="Distance between objects")

    def execute(self, context):
        master = context.active_object
        slaves = context.selected_objects
        to_cadorigin = self.cursor_location - master.matrix_world.to_translation()
        com_shift_world = to_cadorigin + self.com_shift
        for s in slaves:
            sUtils.selectObjects([s], True, 0)
            context.scene.cursor_location = s.matrix_world.to_translation() + com_shift_world
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        sUtils.selectObjects(slaves, True, slaves.index(master))
        context.scene.cursor_location = self.cursor_location.copy()
        return {'FINISHED'}

    def invoke(self, context, event):
        self.cursor_location = context.scene.cursor_location.copy()
        return self.execute(context)

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class CreateInertialOperator(Operator):
    """Create inertial objects based on existing objects"""
    bl_idname = "object.create_inertial_objects"
    bl_label = "Create Inertials"
    bl_options = {'REGISTER', 'UNDO'}

    auto_compute = BoolProperty(
        name='Calculate Automatically',
        default=True,
        description='Calculate inertia automatically'
    )

    preserve_children = BoolProperty(
        name='Preserve Child Inertials',
        default=False,
        description='Preserve child inertials'
    )

    def execute(self, context):
        links = [obj for obj in context.selected_objects if obj.phobostype == 'link']
        show_progress = bpy.app.version[0] * 100 + bpy.app.version[1] >= 269
        if show_progress:
            wm = context.window_manager
            total = float(len(links))
            wm.progress_begin(0, total)
            i = 1
        for link in links:
            inertia.createInertials(link, not self.auto_compute, self.preserve_children)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}


class AddGravityVector(Operator):
    """Add a vector representing gravity in the scene"""
    bl_idname = "object.phobos_add_gravity"
    bl_label = "Add Gravity"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = FloatVectorProperty(
        name="Gravity Vector",
        default=(0, 0, -9.81),
        description="Gravity vector")

    def execute(self, context):
        bpy.ops.object.empty_add(type='SINGLE_ARROW')
        context.active_object.name = "gravity"
        bpy.ops.transform.rotate(value=(math.pi), axis=(1.0, 0.0, 0.0))
        return {'FINISHED'}


class EditYAMLDictionary(Operator):
    """Edit object dictionary as YAML"""
    bl_idname = 'object.phobos_edityamldictionary'
    bl_label = "Edit YAML Dictionary"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        startLog(self)
        ob = context.active_object
        textfilename = ob.name + datetime.now().strftime("%Y%m%d_%H:%M")
        variablename = ob.name.translate({ord(c): "_" for c in "!@#$%^&*()[]{};:,./<>?\|`~-=+"}) \
                       + "_data"
        tmpdict = dict(ob.items())
        for key in tmpdict:
            if hasattr(tmpdict[key], 'to_list'):  # transform Blender id_arrays into lists
                tmpdict[key] = list(tmpdict[key])
        contents = [variablename + ' = """',
                    yaml.dump(bUtils.cleanObjectProperties(tmpdict),
                              default_flow_style=False) + '"""\n',
                    "# ------- Hit 'Run Script' to save your changes --------",
                    "import yaml", "import bpy",
                    "tmpdata = yaml.load(" + variablename + ")",
                    "for key in dict(context.active_object.items()):",
                    "   del context.active_object[key]",
                    "for key, value in tmpdata.items():",
                    "    context.active_object[key] = value",
                    "bpy.ops.text.unlink()"
                    ]
        bUtils.createNewTextfile(textfilename, '\n'.join(contents))
        bUtils.openScriptInEditor(textfilename)
        endLog()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class CreateCollisionObjects(Operator):
    """Create collision objects for all selected links"""
    bl_idname = "object.create_collision_objects"
    bl_label = "Create Collision Objects"
    bl_options = {'REGISTER', 'UNDO'}

    property_colltype = EnumProperty(
        name='Collision Type',
        default='box',
        description="Collision type",
        items=defs.geometrytypes)

    def execute(self, context):

        startLog(self)
        visuals = []
        for obj in context.selected_objects:
            if obj.phobostype == "visual":
                visuals.append(obj)
            obj.select = False

        if not visuals:
            # bpy.ops.error.message('INVOKE_DEFAULT', type="CreateCollisions Error", message="Not enough bodies selected.")
            log("Not enough bodies selected.", "ERROR")
            return {'CANCELLED'}
        for vis in visuals:
            nameparts = vis.name.split('_')
            if nameparts[0] == 'visual':
                nameparts[0] = 'collision'
            collname = '_'.join(nameparts)
            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"
            bBox = vis.bound_box
            center = gUtils.calcBoundingBoxCenter(bBox)
            rotation = mathutils.Matrix.Identity(4)
            size = list(vis.dimensions)
            if self.property_colltype in ['cylinder', 'capsule']:
                axes = ('X', 'Y', 'Z')
                long_side = axes[size.index(max(size))]
                # xyequal = (size[0] - size[1])
                length = max(size)
                radii = [s for s in size if s != length]
                radius = max(radii) / 2 if radii != [] else length / 2
                size = (radius, length)
                if long_side == 'X':
                    rotation = mathutils.Matrix.Rotation(math.pi / 2, 4, 'Y')
                elif long_side == 'Y':
                    rotation = mathutils.Matrix.Rotation(math.pi / 2, 4, 'X')
                    # FIXME: apply rotation for moved cylinder object?
            elif self.property_colltype == 'sphere':
                size = max(size) / 2
            rotation_euler = (vis.matrix_world * rotation).to_euler()
            center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion() * center
            if self.property_colltype != 'capsule' and self.property_colltype != 'mesh':
                ob = bUtils.createPrimitive(collname, self.property_colltype, size,
                                             defs.layerTypes['collision'], materialname, center,
                                             rotation_euler)
            elif self.property_colltype == 'capsule':
                length = max(length - 2 * radius, 0.001)  # prevent length from turning negative
                size = (radius, length)
                zshift = length / 2
                tmpsph1_location = center + rotation_euler.to_matrix().to_4x4() * mathutils.Vector((0,0,zshift))
                tmpsph2_location = center - rotation_euler.to_matrix().to_4x4() * mathutils.Vector((0,0,zshift))
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
                ob['geometry/length'] = length
                ob['geometry/radius'] = radius
                ob['sph1_location'] = tmpsph1_location
                ob['sph2_location'] = tmpsph2_location
            elif self.property_colltype == 'mesh':
                bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False, "mode":'TRANSLATION'}, TRANSFORM_OT_translate={"value":(0, 0, 0), "constraint_axis":(False, False, False), "constraint_orientation":'GLOBAL', "mirror":False, "proportional":'DISABLED', "proportional_edit_falloff":'SMOOTH', "proportional_size":1, "snap":False, "snap_target":'CLOSEST', "snap_point":(0, 0, 0), "snap_align":False, "snap_normal":(0, 0, 0), "gpencil_strokes":False, "texture_space":False, "remove_on_cancel":False, "release_confirm":False})

                # TODO: copy mesh!!
            ob.phobostype = 'collision'
            ob['geometry/type'] = self.property_colltype
            if vis.parent:
                ob.select = True
                bpy.ops.object.transform_apply(scale=True)
                vis.parent.select = True
                context.scene.objects.active = vis.parent
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                # ob.parent_type = vis.parent_type
                # ob.parent_bone = vis.parent_bone
        endLog()
        return {'FINISHED'}


class SetCollisionGroupOperator(Operator):
    """Set the collision groups of the selected object(s)"""
    bl_idname = "object.phobos_set_collision_group"
    bl_label = "Set Collision Groups"
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
        except AttributeError:
            pass  # TODO: catch properly
        return self.execute(context)

    def execute(self, context):
        """This function executes this blender operator and sets the collision groups for the selected object(s).

        :param context: The blender context this operator should work with.
        :return: set -- the blender specific return set.

        """
        objs = filter(lambda e: "phobostype" in e and e.phobostype == "collision", context.selected_objects)
        active_object = context.active_object
        for obj in objs:
            try:
                obj.rigid_body.collision_groups = self.groups
            except AttributeError:
                context.scene.objects.active = obj
                bpy.ops.rigidbody.object_add(type='ACTIVE')
                obj.rigid_body.kinematic = True
                obj.rigid_body.collision_groups = self.groups
        context.scene.objects.active = active_object
        return {'FINISHED'}


class DefineJointConstraintsOperator(Operator):
    """Add bone constraints to the joint (link)"""
    bl_idname = "object.define_joint_constraints"
    bl_label = "Define Joint Constraints"
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

        lower = 0
        upper = 0
        if self.joint_type in ('revolute', 'prismatic'):
            if not self.useRadian:
                lower = math.radians(self.lower)
                upper = math.radians(self.upper)
            else:
                lower = self.lower
                upper = self.upper
        if not self.useRadian:
            velocity = self.maxvelocity * ((2 * math.pi) / 360)  # from °/s to rad/s
        else:
            velocity = self.maxvelocity
        for joint in (obj for obj in context.selected_objects if obj.phobostype == 'link'):
            context.scene.objects.active = joint
            joints.setJointConstraints(joint, self.joint_type, lower, upper, self.spring, self.damping)
            if self.joint_type != 'fixed':
                joint['joint/maxeffort'] = self.maxeffort
                joint['joint/maxvelocity'] = velocity
            else:
                if "joint/maxeffort" in joint: del joint["joint/maxeffort"]
                if "joint/maxvelocity" in joint: del joint["joint/maxvelocity"]
            if self.passive:
                joint['joint/passive'] = "$true"
            else:
                log("Please add motor to active joint in " + joint.name, "INFO", "DefineJointConstraintsOperator")
        return {'FINISHED'}


class AttachMotorOperator(Operator):
    """Attach motor values to selected joints"""
    bl_idname = "object.attach_motor"
    bl_label = "Attach Motor"
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
        default='PID',
        description="Type of the motor",
        items=defs.motortypes)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "motortype", text="motor_type")
        layout.prop(self, "taumax", text="maximum torque [Nm]")
        layout.prop(self, "vmax", text="maximum velocity [m/s] or [rad/s]")
        if self.motortype == 'PID':
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

        objs = filter(lambda e: "phobostype" in e and e.phobostype == "link", context.selected_objects)
        for joint in objs:
            # TODO: these keys have to be adapted
            if self.motortype == 'PID':
                joint['motor/p'] = self.P
                joint['motor/i'] = self.I
                joint['motor/d'] = self.D
            joint['motor/maxSpeed'] = self.vmax
            joint['motor/maxEffort'] = self.taumax
            # joint['motor/type'] = 'PID' if self.motortype == 'PID' else 'DC'
            joint['motor/type'] = self.motortype
        return {'FINISHED'}


class CreateLinkOperator(Operator):
    """Create link(s), optionally based on existing objects"""
    bl_idname = "object.phobos_create_link"
    bl_label = "Create Links"
    bl_options = {'REGISTER', 'UNDO'}

    type = EnumProperty(
        items=(('3D cursor',) * 3,
               ('selected objects',) * 3),
        default='selected objects',
        name='Location',
        description='Where to create new link(s)?'
    )

    size = FloatProperty(
        name="Visual Link Size",
        default=0.2,
        description="Size of the created link"
    )

    parenting = BoolProperty(
        name='Parenting',
        default=False,
        description='Parent associated objects to created links?'
    )

    parentobject = BoolProperty(
        name='Parent Object(s)',
        default=False,
        description='Parent objects to newly created links?'
    )

    namepartindices = StringProperty(
        name="Name Segment Indices",
        description="Allow reusing parts of objects' names, specified as e.g. '2 3'",
        default=''
    )

    separator = StringProperty(
        name="Separator",
        description="Separator to split object names with, e.g. '_'",
        default='_'
    )

    prefix = StringProperty(
        name="Prefix",
        description="Prefix to put before names, e.g. 'link'",
        default='link'
    )

    def execute(self, context):

        if self.type == '3D cursor':
            links.createLink(self.size)
        else:
            for obj in context.selected_objects:
                tmpnamepartindices = [int(p) for p in self.namepartindices.split()]
                links.deriveLinkfromObject(obj, scale=self.size, parenting=self.parenting, parentobjects=self.parentobject,
                                     namepartindices=tmpnamepartindices, separator=self.separator,
                                     prefix=self.prefix)
        return {'FINISHED'}


class AddSensorOperator(Operator):
    """Add/update a sensor"""
    bl_idname = "object.phobos_add_sensor"
    bl_label = "Add/Update A Sensor"
    bl_options = {'REGISTER', 'UNDO'}

    sensor_type = EnumProperty(
        name="Sensor Type",
        default="undefined",
        items=tuple([(type,) * 3 for type in defs.sensortypes]),
        description="Type of the sensor to be created"
    )

    custom_type = StringProperty(
        name="Custom Type",
        default='',
        description="Type of the custom sensor to be created"
    )

    sensor_name = StringProperty(
        name="Sensor Name",
        default='new_sensor',
        description="Name of the sensor"
    )

    add_link = BoolProperty(name="add_link", default=True, description="add additional link as sensor mounting")

    # the following is a set of all properties that exist within MARS' sensors
    # TODO: we should get rid of gui-settings such as hud and rename stuff (eg. maxDistance - maxDist)
    width = IntProperty(name='Width', default=0, description='Width')
    height = IntProperty(name='Height', default=0, description='Height')
    resolution = FloatProperty(name='Resolution', default=0, description='Resolution')
    horizontal_resolution = FloatProperty(name='Horizontal Resolution', default=0, description='Horizontal resolution')
    opening_width = FloatProperty(name='Opening Width', default=0, description='Opening width')
    opening_height = FloatProperty(name='Opening Height', default=0, description='Opening height')
    maxDistance = FloatProperty(name='Max Distance', default=0, description='Maximum distance')
    maxDist = FloatProperty(name='Max Dist', default=0, description='Max dist')
    verticalOpeningAngle = FloatProperty(name='Vertical Opening Angle', default=0, description='Vertical opening angle')
    horizontalOpeningAngle = FloatProperty(name='Horizontal Opening Angle', default=0,
                                           description='horizontal opening angle')
    hud_pos = IntProperty(name='HUD Position', default=0, description='HUD position')
    hud_width = IntProperty(name='HUD Width', default=0, description='HUD width')
    hud_height = IntProperty(name='HUD Height', default=0, description='HUD height')
    updateRate = FloatProperty(name='Update Rate', default=0, description='Update rate')
    vertical_offset = FloatProperty(name='Vertical Offset', default=0, description='Vertical offset')
    horizontal_offset = FloatProperty(name='Horizontal Offset', default=0, description='Horizontal offset')
    gain = FloatProperty(name='Gain', default=0, description='Gain')
    left_limit = FloatProperty(name='Left Limit', default=0, description='Left limit')
    right_limit = FloatProperty(name='Right Limit', default=0, description='Right limit')
    rttResolutionX = FloatProperty(name='RTT Resolution x', default=0, description='RTT resolution x')
    rttResolutionY = FloatProperty(name='RTT Resolution y', default=0, description='RTT resolution y')
    numRaysHorizontal = FloatProperty(name='Number Rays Horizontal', default=0, description='Number of horizontal rays')
    numRaysVertical = FloatProperty(name='Number Rays Vertical', default=0, description='Number of vertical rays')
    draw_rays = BoolProperty(name='Draw Rays', default=False, description='Draw rays')
    depthImage = BoolProperty(name='Depth Image', default=False, description='Depth of the image')
    show_cam = BoolProperty(name='Show Camera', default=False, description='Show the camera?')
    only_ray = BoolProperty(name='Only Ray', default=False, description='Only ray')
    ping_pong_mode = BoolProperty(name='Ping Pong Mode', default=False, description='Ping pong mode')
    bands = IntProperty(name='Bands', default=0, description='Bands')
    lasers = IntProperty(name='Lasers', default=0, description='Lasers')
    extension = FloatVectorProperty(name='Extension', default=(0, 0, 0), description='Extension')

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "sensor_name", text="Sensor Name")
        layout.prop(self, "sensor_type", text="Sensor Type")
        if self.sensor_type in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                                'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            layout.prop(self, "add_link", "Attach to New Link")
        if self.sensor_type == "custom":
            layout.prop(self, "custom_type", text="Custom Type")
        else:
            for key in defs.sensorProperties[self.sensor_type]:
                layout.prop(self, key, text=key)

    def execute(self, context):
        # create a dictionary holding the sensor definition
        sensor = {'name': self.sensor_name,
                  'type': self.custom_type if self.sensor_type == 'Custom' else self.sensor_type,
                  'props': {}
                  }
        parent = context.active_object
        for key in defs.sensorProperties[self.sensor_type]:
            if type(defs.sensorProperties[self.sensor_type][key]) == type(True):
                value = getattr(self, key)
                sensor['props'][key] = '$true' if value else '$false'
            else:
                sensor['props'][key] = getattr(self, key)
        # type-specific settings
        if sensor['type'] in ['CameraSensor', 'ScanningSonar', 'RaySensor',
                              'MultiLevelLaserRangeFinder', 'RotatingRaySensor']:
            if self.add_link:
                link = links.createLink(scale=0.1, position=context.active_object.matrix_world.to_translation(),
                                        name='link_' + self.sensor_name)
                sensorObj = sensors.createSensor(sensor, link, link.matrix_world)
            else:
                sensorObj = sensors.createSensor(sensor, context.active_object, context.active_object.matrix_world)
            if self.add_link:
                sUtils.selectObjects([parent, link], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
                sUtils.selectObjects([link, sensorObj], clear=True, active=0)
                bpy.ops.object.parent_set(type='BONE_RELATIVE')
            sensors.cameraRotLock(sensorObj)
        elif sensor['type'] in ['Joint6DOF']:
            for obj in context.selected_objects:
                if obj.phobostype == 'link':
                    sensor['name'] = "sensor_joint6dof_" + nUtils.getObjectName(obj, phobostype="joint")
                    sensors.createSensor(sensor, obj, obj.matrix_world)
        elif 'Node' in sensor['type']:
            sensors.createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'collision'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        elif 'Motor' in sensor['type'] or 'Joint' in sensor['type']:
            sensors.createSensor(sensor, [obj for obj in context.selected_objects if obj.phobostype == 'link'],
                         mathutils.Matrix.Translation(context.scene.cursor_location))
        return {'FINISHED'}


class CreateMimicJointOperator(Operator):
    """Make a number of joints follow a specified joint"""
    bl_idname = "object.phobos_create_mimic_joint"
    bl_label = "Create Mimic Joint"
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
        for obj in context.selected_objects:
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
        return (ob is not None and ob.phobostype == 'link'
                and len(context.selected_objects) > 1)


class RefineLevelOfDetailOperator(Operator):
    """Refine LoD settings with minimum distances"""
    bl_idname = "object.phobos_refine_lod"
    bl_label = "Refine Level of Detail"
    bl_options = {'REGISTER', 'UNDO'}

    maxdistances = FloatVectorProperty(name="Maximum Distances",
        description="Maximum distances", size=5)

    mindistances = FloatVectorProperty(name="Minimum Distances",
        description="Minimum distances", size=5)

    def draw(self, context):
        layout = self.layout
        inlayout = layout.split()
        c1 = inlayout.column(align=True)
        c2 = inlayout.column(align=True)
        c3 = inlayout.column(align=True)

        lodlist = [lod.object.data.name for lod in context.active_object.lod_levels]
        while len(lodlist) < len(self.maxdistances):
            lodlist.append('not assigned')

        c1.label("Level of Detail Objects:")
        for lodname in lodlist:
            c1.label(text=lodname)
        c2.prop(self, 'mindistances')
        c3.prop(self, 'maxdistances')

    def invoke(self, context, event):
        #nlod = len(context.active_object.lod_levels)
        #self.startdistances = FloatVectorProperty(name="startDistances", description="minimum distances", size=nlod)
        #self.enddistances = FloatVectorProperty(name="endDistances", description="maximum distances", size=nlod)
        lodlist = [0.0]*5
        for lod in range(min(5, len(context.active_object.lod_levels))):
            lodlist[lod] = context.active_object.lod_levels[lod].distance
        self.mindistances = tuple(lodlist)
        lodlist = [0.0]*5
        for lod in range(min(4, len(context.active_object.lod_levels)-1)):
            lodlist[lod] = context.active_object.lod_levels[lod+1].distance
        self.maxdistances = tuple(lodlist)

    #    obj = context.active_object
    #    #self.mindistances = tuple(obj[a] for a in obj.keys() if a.startswith('visual/lod'))
    #    self.mindistances = tuple(lod.distance for lod in obj.lod_levels)
        return self.execute(context)

    def execute(self, context):
        sourceobj = context.active_object
        selobjects = context.selected_objects
        n = len(sourceobj.lod_levels)
        for obj in selobjects:
            if obj.phobostype in ['visual', 'collision']:
                if obj != sourceobj:
                    sUtils.selectObjects([obj], clear=True, active=0)
                    bpy.ops.object.lod_clear_all()
                    for i in range(n-1):
                        bpy.ops.object.lod_add()
                        obj.lod_levels[i+1].distance = sourceobj.lod_levels[i+1].distance
                        obj.lod_levels[i+1].object = sourceobj.lod_levels[i+1].object
                for i in range(n):
                    lodlist = []
                    #loddict = {'start': self.startdistances[dist], 'end': self.enddistances[dist],
                    #           'filename': sourceobj.lod_levels[dist].object.data.name}
                    #obj['lod/' + str(dist) + '_start'] = self.startdistances[dist]
                    obj.lod_levels[i].distance = self.mindistances[i]
                    #obj['lod/' + str(dist) + '_end'] = sourceobj.lod_levels[dist].distance
                    #obj['lod/' + str(dist) + '_mesh'] = sourceobj.lod_levels[dist].object.data.name
                    #obj['lod/lod'] = loddict
                obj['lodmaxdistances'] = list(self.maxdistances[:len(obj.lod_levels)])
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.phobostype == 'visual'


class AddHeightmapOperator(Operator):
    """Add a heightmap object to the 3D-Cursors location"""
    bl_idname = "object.phobos_add_heightmap"
    bl_label = "Adds a heightmap object to the 3D-Cursors location"
    bl_options = {'REGISTER', 'UNDO'}

    name = StringProperty(
        name="Name",
        description="The new heightmap's name",
        default="heightmap"
    )

    cutNo = IntProperty(
        name="Number of Cuts",
        description="Number of cuts for subdivide",
        default=100
    )

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        startLog(self)
        if os.path.basename(self.filepath) not in bpy.data.images:
            try:
                img = bpy.data.images.load(self.filepath)
            except RuntimeError:
                log("Cannot load image from file! Aborting.", "ERROR")
                return {"FINISHED"}
        else:
            log("Image already imported. Using cached version.", "INFO")
            img = bpy.data.images[os.path.basename(self.filepath)]
        # Create Texture
        h_tex = bpy.data.textures.new(self.name, type='IMAGE')
        h_tex.image = img
        # Add plane, subdivide and create displacement
        prev_mode = context.mode
        bpy.ops.mesh.primitive_plane_add(view_align=False, enter_editmode=False)
        plane = context.active_object
        plane['phobostype'] = 'visual'
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.subdivide(number_cuts=self.cutNo)
        bpy.ops.object.mode_set(mode=prev_mode)
        plane.modifiers.new('displace_heightmap', 'DISPLACE')
        plane.modifiers['displace_heightmap'].texture = h_tex
        plane.name = self.name+'_visual::heightmap'
        # Add root link for heightmap
        root = links.createLink(1.0, name=self.name + "::heightmap")
        root['entity/type'] = 'heightmap'
        root['entity/name'] = self.name
        root['image'] = os.path.relpath(os.path.basename(self.filepath), bpy.data.filepath)
        root['joint/type'] = 'fixed'
        # Create Parenting
        sUtils.selectObjects([root, plane], clear=True, active=0)
        bpy.ops.object.parent_set(type='BONE_RELATIVE')
        endLog()
        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}


def add_editing_manual_map():
    """This allows you to right click on a button and link to the manual

    :return: tuple

    """
    url_manual_prefix = "https://github.com/rock-simulation/phobos/wiki/Operators#"
    url_manual_mapping = (
        ("bpy.ops.object.phobos_sort_objects_to_layers", "set-objects-to-layers"),
        ("bpy.ops.object.phobos_add_chain", "define-kinematic-chain"),
        ("bpy.ops.object.phobos_set_mass", "set-mass"),
        ("bpy.ops.object.phobos_sync_masses", "sync-masses"),
        ("bpy.ops.object.phobos_set_xray", "x-ray-view"),
        ("bpy.ops.object.phobos_set_phobostype", "set-phobostype"),
        ("bpy.ops.object.phobos_batch_property", "edit-custom-property"),
        ("bpy.ops.object.phobos_copy_props", "copy-custom-property"),
        ("bpy.ops.object.phobos_rename_custom_property", "rename-custom-property"),
        ("bpy.ops.object.phobos_set_geometry_type", "set-geometry-types"),
        ("bpy.ops.object.phobos_edit_inertia", "edit-inertia"),
        ("bpy.ops.object.phobos_smoothen_surface", "smoothen-surface"),
        ("bpy.ops.object.phobos_set_origin_to_com", "set-origin-to-com"),
        ("bpy.ops.object.create_inertial_objects", "create-inertial-objects"),
        ("bpy.ops.object.phobos_add_gravity", ""),
        ("bpy.ops.object.phobos_edityamldictionary", "edit-object-dictionary"),
        ("bpy.ops.object.create_collision_objects", "create-collision-objects"),
        ("bpy.ops.object.phobos_set_collision_group", "set-collision-group"),
        ("bpy.ops.object.define_joint_constraints", "define-joint-constraints"),
        ("bpy.ops.object.attach_motor", "attach-motor"),
        ("bpy.ops.object.phobos_create_link", "create-links"),
        ("bpy.ops.object.phobos_add_sensor", "addedit-sensor"),
        ("bpy.ops.object.phobos_create_mimic_joint", "mimic-joint"),
        ("bpy.ops.object.phobos_refine_lod", "refine-lod"),
        ("object.phobos_add_heightmap", "add-heightmap"),
    )
    return url_manual_prefix, url_manual_mapping


def register():
    """This function is called when this module is registered to blender.

    """
    print("Registering operators.editing...")
    bpy.utils.register_manual_map(add_editing_manual_map)
    bpy.utils.register_class(SortObjectsToLayersOperator)
    bpy.utils.register_class(AddChainOperator)
    bpy.utils.register_class(SetMassOperator)
    bpy.utils.register_class(SyncMassesOperator)
    bpy.utils.register_class(SetXRayOperator)
    bpy.utils.register_class(SetPhobosType)
    bpy.utils.register_class(BatchEditPropertyOperator)
    bpy.utils.register_class(CopyCustomProperties)
    bpy.utils.register_class(RenameCustomProperty)
    bpy.utils.register_class(SetGeometryType)
    bpy.utils.register_class(EditInertia)
    bpy.utils.register_class(SmoothenSurfaceOperator)
    bpy.utils.register_class(SetOriginToCOMOperator)
    bpy.utils.register_class(CreateInertialOperator)
    bpy.utils.register_class(AddGravityVector)
    bpy.utils.register_class(EditYAMLDictionary)
    bpy.utils.register_class(CreateCollisionObjects)
    bpy.utils.register_class(SetCollisionGroupOperator)
    bpy.utils.register_class(DefineJointConstraintsOperator)
    bpy.utils.register_class(AttachMotorOperator)
    bpy.utils.register_class(CreateLinkOperator)
    bpy.utils.register_class(AddSensorOperator)
    bpy.utils.register_class(CreateMimicJointOperator)
    bpy.utils.register_class(RefineLevelOfDetailOperator)
    bpy.utils.register_class(AddHeightmapOperator)


def unregister():
    """ This function is called when this module is unregistered from blender.

    """
    print("Unregistering operators.editing...")
    bpy.utils.unregister_manual_map(add_editing_manual_map)
    bpy.utils.unregister_class(SortObjectsToLayersOperator)
    bpy.utils.unregister_class(AddChainOperator)
    bpy.utils.unregister_class(SetMassOperator)
    bpy.utils.unregister_class(SyncMassesOperator)
    bpy.utils.unregister_class(SetXRayOperator)
    bpy.utils.unregister_class(SetPhobosType)
    bpy.utils.unregister_class(BatchEditPropertyOperator)
    bpy.utils.unregister_class(CopyCustomProperties)
    bpy.utils.unregister_class(RenameCustomProperty)
    bpy.utils.unregister_class(SetGeometryType)
    bpy.utils.unregister_class(EditInertia)
    bpy.utils.unregister_class(SmoothenSurfaceOperator)
    bpy.utils.unregister_class(SetOriginToCOMOperator)
    bpy.utils.unregister_class(CreateInertialOperator)
    bpy.utils.unregister_class(AddGravityVector)
    bpy.utils.unregister_class(EditYAMLDictionary)
    bpy.utils.unregister_class(CreateCollisionObjects)
    bpy.utils.unregister_class(SetCollisionGroupOperator)
    bpy.utils.unregister_class(DefineJointConstraintsOperator)
    bpy.utils.unregister_class(AttachMotorOperator)
    bpy.utils.unregister_class(CreateLinkOperator)
    bpy.utils.unregister_class(AddSensorOperator)
    bpy.utils.unregister_class(CreateMimicJointOperator)
    bpy.utils.unregister_class(RefineLevelOfDetailOperator)
    bpy.utils.unregister_class(AddHeightmapOperator)
