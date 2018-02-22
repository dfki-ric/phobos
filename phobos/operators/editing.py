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
import phobos.model.inertia as inertia
import phobos.utils.selection as sUtils
import phobos.utils.general as gUtils
import phobos.utils.blender as bUtils
import phobos.utils.naming as nUtils
import phobos.utils.editing as eUtils
import phobos.model.joints as joints
import phobos.model.links as links
from phobos.phoboslog import log


class SortObjectsToLayersOperator(Operator):
    """Sort all selected objects to their according layers"""
    bl_idname = "phobos.sort_objects_to_layers"
    bl_label = "Sort Objects to Layers"
    bl_options = {'UNDO'}

    def execute(self, context):
        objs = filter(lambda e: "phobostype" in e, context.selected_objects)
        # TODO maybe clear layers first of all objects without phobostype?
        for obj in objs:
            phobosType = obj.phobostype
            # TODO what about controllers?
            # sort phobostypes to layers defined in defs
            if phobosType != 'controller' and phobosType != 'undefined':
                layers = 20 * [False]
                layers[defs.layerTypes[phobosType]] = True
                obj.layers = layers

            # undefined type will be shown in statusbar
            if phobosType == 'undefined':
                log("The phobostype of the object '" + obj.name + "' is" + "undefined", "ERROR")
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


class SetMassOperator(Operator):
    """Set the mass of the selected object(s)"""
    bl_idname = "phobos.set_mass"
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
        return (bpy.context.active_object is not None and
                len([e.phobostype in ("visual", "collision", "inertial")
                     for e in context.selected_objects]) > 0)

    def invoke(self, context, event):
        if 'mass' in context.active_object:
            self.mass = context.active_object['mass']
        return self.execute(context)

    def execute(self, context):
        objs = [obj for obj in bpy.context.selected_objects
                if obj.phobostype in ("visual", "collision", "inertial")]
        for obj in objs:
            # check for old mass value
            try:
                oldmass = obj['mass']
            except KeyError:
                oldmass = None

            # use rigid body mass
            if self.userbmass:
                try:
                    obj['mass'] = obj.rigid_body.mass
                except AttributeError:
                    obj['mass'] = 0.001
                    log("The object '" + obj.name + "' has no rigid body" +
                        "properties. Set mass to 0.001", "ERROR")
            # use provided mass
            else:
                obj['mass'] = self.mass

            # only keep oldmass when it exists
            if obj['mass'] != oldmass and oldmass:
                t = datetime.now()
                obj['masschanged'] = t.isoformat()
        return {'FINISHED'}

    def invoke(self, context, event):
        if 'mass' in context.active_object:
            self.mass = context.active_object['mass']
        return self.execute(context)


class SyncMassesOperator(Operator):
    """Synchronize masses among the selected object(s)"""
    bl_idname = "phobos.sync_masses"
    bl_label = "Sync Masses"
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
        # FIXME NoneTypeError although objects are selected
        links = [obj.name for obj in context.selected_objects if obj.phobostype == 'link']
        t = datetime.now()
        objdict = {obj.name: obj for obj in bpy.data.objects if obj.phobostype in ['visual', 'collision'] and
                   obj.parent.name in links}

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
                    # if collision information is older than visual information
                    if tc < tv:
                        sourcelist.append('visual_' + basename)
                        targetlist.append('collision_' + basename)
                    else:
                        targetlist.append('visual_' + basename)
                        sourcelist.append('collision_' + basename)
                except KeyError:
                    log(basename + " has insufficient data for time-based" +
                        "synchronisation of masses.", "WARNING")

        # sync the mass values
        for i in range(len(sourcelist)):
            try:
                objdict[targetlist[i]]['mass'] = objdict[sourcelist[i]]['mass']
            except KeyError:
                log("No mass information in object " + targetlist[i], "WARNING")
            if self.synctype != "vtc" and self.synctype != "ctv":
                objdict[targetlist[i]]['masschanged'] = objdict[sourcelist[i]]['masschanged']

        # update inertials and sum masses for links
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

    @classmethod
    def poll(cls, context):
        return len(context.selected_objects) > 0


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
    """Edit phobostype of selected object(s)"""
    bl_idname = "phobos.set_phobostype"
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
        return len(context.selected_objects) > 0 and (
                context.selected_objects[0].mode == 'OBJECT')

    def invoke(self, context, event):
        # take phobostype from active object
        if 'phobostype' in context.active_object:
            objtype = context.active_object['phobostype']
            self.phobostype = defs.phobostypes[objtype][0]
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
                log("The object '" + obj.name + "' is no collision or visual.",
                   "INFO")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class EditInertia(Operator):
    """Edit inertia of selected object(s)"""
    bl_idname = "phobos.edit_inertia"
    bl_label = "Edit Inertia"
    bl_options = {'REGISTER', 'UNDO'}

    inertiavector = FloatVectorProperty(
        name="Inertia Vector",
        default=[0, 0, 0, 0, 0, 0],
        subtype='NONE',
        size=6,
        description="Set inertia for a link"
    )

    def invoke(self, context, event):
        # use inertia of active object
        if 'inertia' in context.active_object:
            self.inertiavector = mathutils.Vector(context.active_object['inertia'])
        return self.execute(context)

    def execute(self, context):
        objs = filter(lambda e: "phobostype" in e and e.phobostype == "inertial", context.selected_objects)
        for obj in objs:
            obj['inertia'] = self.inertiavector
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and ob.phobostype == 'inertial' and len(
            context.selected_objects) > 0


class SmoothenSurfaceOperator(Operator):
    """Smoothen surface of selected objects"""
    bl_idname = "phobos.smoothen_surface"
    bl_label = "Smoothen Surface"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        show_progress = bUtils.getBlenderVersion() >= 269
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


class CreateLinkInertialOperator(Operator):
    """Create inertial object(s) for link(s)"""
    bl_idname = "phobos.create_link_inertials"
    bl_label = "Create link inertia"
    bl_options = {'REGISTER', 'UNDO'}

    def getMajorInertials(self, context, link=None):
        """
        Returns all link inertials of the selected links (in the current
        context) or of the specified link.

        :param context: Blender context
        :param link: the link of which to find the inertials (optional)

        :return: dictionary of links with list of Blender objects or just a list.
        """
        links = [obj for obj in context.selected_objects
                 if obj.phobostype == 'link']

        major_inertials = {}
        # append the link inertials for each link
        if link:
            inertials = sUtils.getImmediateChildren(
                link, phobostypes=('inertial'), include_hidden=True)
            major_inertials = [inert for inert in inertials if 'inertial/inertia' in inert]

        else:
            for link in links:
                linkname = link['link/name']
                inertials = sUtils.getImmediateChildren(
                    link, phobostypes=('inertial'), include_hidden=True)
                major_inertials[linkname] = [inert for inert in inertials if 'inertial/inertia' in inert]

        return major_inertials

    from_selected_only = BoolProperty(
        name='From selected objects', default=False,
        description='Include only the selected ' +
        'visual/collision object(s) into the link inertial(s).'
    )

    autocalc = BoolProperty(
        name='Calculate automatically',
        default=True,
        description='Calculate inertial(s) automatically. Otherwise' +
                    ' create objects with empty inertia data.'
    )

    overwrite = BoolProperty(
        name='Overwrite existing',
        default=True,
        description='Replace existing link inertial(s).'
    )

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "from_selected_only")
        layout.prop(self, "autocalc")
        layout.prop(self, "overwrite")

    def execute(self, context):
        # keep the currently selected objects
        links = [obj for obj in context.selected_objects
                 if obj.phobostype == 'link']
        selected = context.selected_objects

        # show progress when available
        show_progress = bUtils.getBlenderVersion() >= 269
        if show_progress:
            wm = context.window_manager
            total = float(len(links))
            wm.progress_begin(0, total)
            i = 1

        # calculate inertial objects for each link
        for link in links:
            # delete inertials which are overwritten
            if self.overwrite:
                major_inertials = self.getMajorInertials(context, link)
                sUtils.selectObjects(major_inertials, clear=True)

                # remove deleted inertials from the selection
                for inert in major_inertials:
                    if inert in selected:
                        selected.remove(inert)

                bpy.ops.object.delete()

            # reselect the initial objects
            sUtils.selectObjects(selected, clear=True)
            # calculate the link inertials
            inertia.createMajorInertialObjects(link, self.autocalc,
                                               self.from_selected_only)
            if show_progress:
                wm.progress_update(i)
                i += 1
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        # only enable button when there are links selected
        links = [obj for obj in context.selected_objects
                 if obj.phobostype == 'link']
        return len(links) > 0


class CreateMinorInertialOperator(Operator):
    """Create minor inertial object(s) from collision/visual objects of a link"""
    bl_idname = "phobos.create_minor_inertials"
    bl_label = "Create minor inertials"
    bl_options = {'REGISTER', 'UNDO'}

    def getMinorInertials(self, context, link=None):
        """
        Returns all minor inertials of the selected links (in the current
        context) or of the specified link.

        :param context: Blender context
        :param link: the link of which to find the inertials (optional)

        :return: dictionary of links with list of Blender objects or just a list.
        """
        if not link:
            links = [obj for obj in context.selected_objects
                     if obj.phobostype == 'link']

        minor_inertials = {}
        # append the link inertials for each link
        if link:
            inertials = sUtils.getImmediateChildren(
                link, phobostypes=('inertial'), include_hidden=True)
            minor_inertials = [inert for inert in inertials if 'inertia' in inert]

        else:
            for link in links:
                linkname = link['link/name']
                inertials = sUtils.getImmediateChildren(
                    link, phobostypes=('inertial'), include_hidden=True)
                minor_inertials[linkname] = [inert for inert in inertials if 'inertia' in inert]

        return minor_inertials

    autocalc = BoolProperty(
        name='Calculate automatically',
        default=True,
        description='Calculate inertial(s) automatically. Otherwise' +
                    ' create objects with empty inertia data.'
    )

    overwrite = BoolProperty(
        name='Overwrite existing',
        default=True,
        description='Replace existing link inertial(s).'
    )

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "autocalc")
        layout.prop(self, "overwrite")

    def execute(self, context):
        # keep currently selected objects
        links = [obj for obj in context.selected_objects
                 if obj.phobostype == 'link']
        selected = context.selected_objects

        # show progress if possible
        show_progress = bUtils.getBlenderVersion() >= 269
        if show_progress:
            wm = context.window_manager
            total = float(len(links))
            wm.progress_begin(0, total)
            i = 1

        # create inertials for each link
        for link in links:
            # delete inertials which are overwritten
            if self.overwrite:
                minor_inertials = self.getMinorInertials(context, link)
                sUtils.selectObjects(minor_inertials, clear=True)

                # remove deleted inertials from the selection
                for inert in minor_inertials:
                    if inert in selected:
                        selected.remove(inert)

                bpy.ops.object.delete()

            # reselect the initial objects
            sUtils.selectObjects(selected, clear=True)

            inertia.createMinorInertialObjects(link, self.autocalc)
            if show_progress:
                wm.progress_update(i)
                i += 1

        # reselect the initial objects
        sUtils.selectObjects(selected, clear=True)
        if show_progress:
            wm.progress_end()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        # only enable button when there are links selected
        links = [obj for obj in context.selected_objects
                 if obj.phobostype == 'link']
        return len(links) > 0


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

            # calculate rotation and center coordinates
            rotation_euler = (vis.matrix_world * rotation).to_euler()
            center = vis.matrix_world.to_translation() + vis.matrix_world.to_quaternion() * center

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
        layout.prop(self, "motortype", text="motor_type")
        if not self.motortype == 'none':
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
        objs = (obj for obj in context.selected_objects if obj.phobostype == "link")
        for joint in objs:
            # add motor properties
            if not self.motortype == 'none':
                # TODO: these keys have to be adapted
                if self.motortype == 'PID':
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

    linktype = EnumProperty(
        items=(('3D cursor',) * 3,
               ('selected objects',) * 3),
        default='selected objects',
        name='Location',
        description='Where to create new link(s)?'
    )

    size = FloatProperty(
        name="Visual Link Size",
        default=1.0,
        description="Size of the created link"
    )

    parenting = BoolProperty(
        name='Parenting hierarchy',
        default=False,
        description='Use parenting hierarchy for links?'
    )

    parentobject = BoolProperty(
        name='Include selected Object(s)',
        default=False,
        description='Add selected objects to link hierarchy?'
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
        if self.linktype == '3D cursor':
            links.createLink({'name': self.name, 'scale': self.size})
        else:
            for obj in context.selected_objects:
                tmpnamepartindices = [int(p) for p in self.namepartindices.split()]
                links.deriveLinkfromObject(obj, scale=self.size, parenting=self.parenting,
                                           parentobjects=self.parentobject,
                                           namepartindices=tmpnamepartindices,
                                           separator=self.separator,
                                           prefix=self.prefix)
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "linktype")
        layout.prop(self, "size")
        layout.prop(self, "parenting")

        # show parentobject only when using parenting hierarchy
        if self.parenting:
            layout.prop(self, "parentobject")

        layout.prop(self, "namepartindices")
        layout.prop(self, "separator")
        layout.prop(self, "prefix")


def getControllerParameters(name):
    """
    Returns the controller parameters for the controller type with the provided
    name.

    :param name: the name of the controller type.
    :type name: str.
    """
    try:
        return defs.definitions['controllers'][name]['parameters'].keys()
    except:
        return []


def getDefaultControllerParameters(scene, context):
    """
    Returns the default controller parameters for the controller of the active
    object.
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
        root = links.deriveLinkfromObject(plane, scale=1.0, parenting=True,
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

    def draw(self, context):
        l = self.layout
        l.prop(self, 'filepath')
        if self.filepath == '':
            l.prop(self, 'annotationtype')
            l.prop(self, 'devicetype')
            b = self.layout.box()
            for key, value in defs.definitions[self.annotationtype][self.devicetype].items():
                b.label(text=key+': '+str(value))


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


class InstantiateAssembly(Operator):
    """Instantiate an assembly"""
    bl_idname = "phobos.instantiate_assembly"
    bl_label = "Instantiate Assembly"
    bl_options = {'REGISTER', 'UNDO'}

    def getAssembliesListForEnumProperty(self, context):
        assemblieslist = [a.name.replace('assembly:', '') for a in bpy.data.groups
                          if a.name.startswith('assembly')]
        return [(a,)*3 for a in assemblieslist]

    assemblyname = EnumProperty(
        name="Assembly name",
        description="Name of the assembly",
        items=getAssembliesListForEnumProperty
    )

    instancename = StringProperty(
        name="Instance name",
        default=''
    )

    def invoke(self, context, event):
        i = 0
        while self.assemblyname+'_'+str(i) in bpy.data.objects:
            i += 1
        self.instancename = self.assemblyname+'_'+str(i)
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    def execute(self, context):
        eUtils.instantiateAssembly(self.assemblyname, self.instancename)
        return {'FINISHED'}


class DefineAssembly(Operator):
    """Instantiate an assembly"""
    bl_idname = "phobos.define_assembly"
    bl_label = "Define Assembly"
    bl_options = {'REGISTER', 'UNDO'}

    assemblyname = StringProperty(
        name="Assembly name",
        description="Name of the assembly",
        default=''
    )

    version = StringProperty(
        name="Version name",
        description="Name of the assembly version",
        default=''
    )

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        eUtils.defineAssembly(self.assemblyname, self.version)
        return {'FINISHED'}


class ToggleInterfaces(Operator):
    """Instantiate an assembly"""
    bl_idname = "phobos.toggle_interfaces"
    bl_label = "Toggle interfaces"
    bl_options = {'REGISTER', 'UNDO'}

    mode = EnumProperty(
        name="Version name",
        description="Name of the assembly version",
        items=(('toggle',) * 3, ('activate',) * 3, ('deactivate',) * 3)
    )

    def execute(self, context):
        eUtils.toggleInterfaces(None, self.mode)
        return {'FINISHED'}


class ConnectInterfacesOperator(Operator):
    """Connects assemblies at interfaces"""
    bl_idname = "phobos.connect_interfaces"
    bl_label = "Connect Interfaces"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        try:
            if (context.active_object is None or len(context.selected_objects) != 2 or not
                    all([obj.phobostype == 'interface' for obj in context.selected_objects])):
                return False
            else:
                parentinterface = bpy.context.active_object
                childinterface = [a for a in bpy.context.selected_objects if a != bpy.context.active_object][0]
                if ((parentinterface['interface/type'] == childinterface['interface/type']) and
                    (parentinterface['interface/direction'] != childinterface['interface/direction']) or
                    (parentinterface['interface/direction'] == 'bidirectional' and
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


def register():
    print("Registering operators.editing...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.register_class(classdef)


def unregister():
    print("Unregistering operators.editing...")
    for key, classdef in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        bpy.utils.unregister_class(classdef)
