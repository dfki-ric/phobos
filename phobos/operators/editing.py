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
import phobos.utils.io as ioUtils
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

        log("Removed {} object{}".format(len(objs_to_delete),
                                         's' if len(objs_to_delete) > 1 else '')
            + " from configuration.", 'INFO')
        log("    Objects: " + str([obj.name for obj in objs_to_delete]), 'DEBUG')
        if objs_to_keep:
            log('Some objects were not removed as they are unique to this scene.', 'WARNING')
            log("    Objects: " + str([obj.name for obj in objs_to_keep]), 'DEBUG')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.selected_objects


class MoveToSceneOperator(Operator):
    """Moves all selected objects to (new) scene"""
    bl_idname = "phobos.move_to_scene"
    bl_label = "Move To Scene"
    bl_options = {'UNDO'}

    def getSceneEnumProperty(self, context):
        scenes = bpy.data.scenes.keys()

        # remove current scene from enum
        scenes.remove(context.scene.name)

        # resources scene is not an export configuration
        if 'resources' in scenes:
            scenes.remove('resources')
        return bUtils.compileEnumPropertyList(scenes)

    scenename = StringProperty(name='Scene Name', default='new',
                               description='Name of the scene to which to add selection')

    scene = EnumProperty(name='Scene', items=getSceneEnumProperty,
                         description='List of available scenes')

    new = BoolProperty(name='New', default=True,
                       description="Create new scene for configuration")

    truecopy = BoolProperty(name='Copy selected', default=False,
                            description="Copy selected objects from active scene")

    link_obdata = BoolProperty(name='Link mesh data', default=True,
                               description="Link mesh data to new objects")

    link_material = BoolProperty(name='Link material data', default=True,
                          description="Link material data to new objects")

    link_texture = BoolProperty(name='Link texture data', default=True,
                          description="Link texture data to new objects")

    remove = BoolProperty(name='Remove selected', default=False,
                          description='Remove selected objects from active scene')

    init = BoolProperty(name='Link current scene', default=True,
                        description="Link all unselected objects from the current scene")


    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def check(self, context):
        if not self.new:
            self.scenename = self.scene
            self.init = False
        return True

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'new', text="New scene")
        box = layout.box()
        if self.new:
            box.prop(self, 'scenename', text="Scene Name")
            box.prop(self, 'init')
        else:
            box.prop(self, 'scene')

        box = layout.box()
        box.prop(self, 'truecopy')
        if self.truecopy:
            box.prop(self, 'link_obdata')
            box.prop(self, 'link_material')
            box.prop(self, 'link_texture')

        box = layout.box()
        box.prop(self, 'remove')

    def execute(self, context):
        moveobjs = context.selected_objects
        oldscene = context.scene

        newscene = bUtils.switchToScene((self.scenename if self.new else self.scene))
        # link all objects from active scene to new scene (initialize it)
        if self.new and self.init:
            for obj in oldscene.objects:
                if obj.name not in newscene.objects:
                    newscene.objects.link(obj)
                    newscene.update()

        # in any case, make sure to move selected objects objects to scene
        for obj in moveobjs:
            if obj.name not in newscene.objects:
                newscene.objects.link(obj)

        # make the objects single user in the new scene
        if self.truecopy:
            # add phobostype/name to make sure the object keeps its name as single user
            for obj in moveobjs:
                name = nUtils.getObjectName(obj)

                # add phobostype/name to make sure the object keeps its name as single user
                if obj.phobostype + '/name' not in obj:
                    obj[obj.phobostype + '/name'] = name

            # make object single user
            sUtils.selectObjects(moveobjs, clear=True)
            bpy.ops.object.make_single_user(type='SELECTED_OBJECTS', object=True,
                                            obdata=not self.link_obdata, material=not self.link_material,
                                            texture=not self.link_texture)
            newobjs = bpy.context.selected_objects

            # prepend Blender name with scenename (phobostype/names are kept anyway)
            for newobj in newobjs:
                newobj.name = newscene.name + '_' + nUtils.getObjectName(newobj)

        # remove objects from active scene and restructure the kinematic tree
        if self.remove:
            for obj in moveobjs:
                oldscene.objects.unlink(obj)

            roots = sUtils.getRoots(scene=newscene)

            for obj in roots:
                sUtils.selectObjects([obj], clear=True, active=0)
                bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
                log("Cleared parent for new object root {}".format(obj.name), 'INFO')

        newscene.layers = bUtils.defLayers(list(range(20)))
        sUtils.selectObjects(newscene.objects, clear=True)
        bpy.ops.phobos.sort_objects_to_layers()
        newscene.update()

        log("{} {} object{} to {}{}export configuration '{}'.".format(
            'Added', len(moveobjs), 's' if len(moveobjs) > 1 else '', 'new ' if self.new else '',
            'initialized ' if self.init else '', self.scenename), 'INFO')
        log("  Objects: " + str([obj.name for obj in moveobjs]), 'DEBUG')
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
        eUtils.sortObjectsToLayers(context.selected_objects)
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
                eUtils.removeProperties(obj, [self.property_name])
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
    bl_options = {'UNDO'}

    geomType = EnumProperty(
        items=defs.geometrytypes,
        name="Type",
        default="box",
        description="Phobos geometry type")

    def execute(self, context):
        objs = context.selected_objects
        for obj in objs:
            if obj.phobostype == 'collision' or obj.phobostype == 'visual':
                obj['geometry/type'] = self.geomType
            else:
                log("The object '" + obj.name + "' is no collision or visual.", 'WARNING')

        log("Changed geometry type for {} object{}".format(len(objs), 's' if len(objs) > 1 else '')
            + " to {}.".format(self.geomType), 'INFO')
        log("    Objects: " + str([obj.name for obj in objs]), 'DEBUG')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0

    def draw(self, context):
        layout = self.layout

        layout.prop(self, 'geomType')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=200)


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
    """Edit mass/inertia of selected object(s)"""
    bl_idname = "phobos.edit_inertial_data"
    bl_label = "Edit Mass/Inertia"
    bl_options = {'UNDO'}

    changeInertia = BoolProperty(
        name="Change inertia",
        default=True,
        description="Change inertia values")

    inertiavector = FloatVectorProperty(
        name="Inertia Vector",
        default=[1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3],
        subtype='NONE',
        precision=10,
        size=6,
        description="New inertia values for the inertial objects")

    changeMass = BoolProperty(
        name="Change mass",
        default=True,
        description="Change mass values")

    mass = FloatProperty(
        name='Mass',
        default=1e-3,
        description="New mass for the inertial objects (in kg)",
        precision=10)

    def invoke(self, context, event):
        # read initial parameter values from active object
        if context.active_object and context.active_object.phobostype == 'inertial':
            errors, *_ = vUtils.validateInertiaData(context.active_object, adjust=True)
            for error in errors:
                error.log()

            self.mass = context.active_object['inertial/mass']
            self.inertiavector = mathutils.Vector(context.active_object['inertial/inertia'])

        return context.window_manager.invoke_props_dialog(self, width=200)

    def execute(self, context):
        if not self.changeMass and not self.changeInertia:
            log("Cancelled inertia/mass editing: nothing to change.", 'INFO')
            return {'CANCELLED'}

        objs = [obj for obj in context.selected_objects if obj.phobostype == "inertial"]
        user_inertia = {'inertia': self.inertiavector, 'mass': self.mass}

        # validate user data
        errors, inertia_dict = vUtils.validateInertiaData(user_inertia, adjust=True)
        for error in errors:
            error.log()

        newmass = inertia_dict['mass']
        newinertia = inertia_dict['inertia']
        # change object properties accordingly
        for obj in objs:
            if self.changeMass:
                obj['inertial/mass'] = newmass
            if self.changeInertia:
                obj['inertial/inertia'] = newinertia

        if self.changeMass:
            log("Changed mass to " + str(newmass) + " for {} objects.".format(len(objs)), 'INFO')
        if self.changeInertia:
            log("Changed inertia to " + str(newinertia) + " for {} objects.".format(len(objs)),
                'INFO')
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout

        layout.prop(self, 'changeMass', toggle=True)
        if self.changeMass:
            layout.prop(self, 'mass')

        layout.separator()
        layout.prop(self, 'changeInertia', toggle=True)
        if self.changeInertia:
            col = layout.column()
            col.prop(self, 'inertiavector')

    def check(self, context):
        return True

    @classmethod
    def poll(cls, context):
        return bool([obj for obj in context.selected_objects if obj.phobostype == 'inertial'])


class GenerateInertialObjectsOperator(Operator):
    """Creates inertial object(s) to add mass and inertia data to links"""
    bl_idname = "phobos.generate_inertial_objects"
    bl_label = "Create Inertials"
    bl_options = {'REGISTER', 'UNDO'}

    mass = FloatProperty(
        name='Mass',
        default=0.001,
        description="Mass (of active object) in kg")

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

    visuals = BoolProperty(
        name="visual",
        default=True,
        description="Use the selected visual objects for inertial creation.")

    collisions = BoolProperty(
        name="collision",
        default=True,
        description="Use the selected visual objects for inertial creation.")

    def invoke(self, context, event):
        geometric_objects = [obj for obj in context.selected_objects
                             if obj.phobostype in ['visual', 'collision']]

        # initialise the geometry parameter correctly
        if not geometric_objects:
            self.derive_inertia_from_geometry = False
        else:
            self.derive_inertia_from_geometry = True

        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, 'clear')
        geometric_objects = [obj for obj in context.selected_objects
                             if obj.phobostype in ['visual', 'collision']]

        if geometric_objects:
            layout.prop(self, 'derive_inertia_from_geometry')
            visuals = [obj for obj in geometric_objects if obj.phobostype == 'visual']
            collisions = [obj for obj in geometric_objects if obj.phobostype == 'collision']

            layout.label("Use geometry objects:")
            row = layout.row(align=True)
            if not visuals:
                row.label('visuals')
            else:
                row.prop(self, 'visuals', toggle=True)
            if not collisions:
                row.label('collisions')
            else:
                row.prop(self, 'collisions', toggle=True)

        layout.prop(self, 'mass')

    def execute(self, context):
        # store previously selected objects
        selection = context.selected_objects
        viscols = [obj for obj in selection if obj.phobostype in ['visual', 'collision']]
        links = list(set([obj.parent for obj in viscols] +
                         [obj for obj in selection if obj.phobostype == 'link']))

        # gather list of objects to generate inertial for
        if self.derive_inertia_from_geometry:
            objectlist = []
            if self.visuals:
                objectlist.extend([obj for obj in viscols if obj.phobostype == 'visual'])
            if self.collisions:
                objectlist.extend([obj for obj in viscols if obj.phobostype == 'collision'])

            if not objectlist:
                log("No objects to create inertial objects from.", 'ERROR')
                return {'CANCELLED'}
        else:
            objectlist = links

        # remove the old inertial objects
        if self.clear:
            inertial_objects = [obj for link in links for obj in link.children
                                if obj.phobostype == 'inertial']
            log("Removing old inertial objects: " + str(inertial_objects), 'DEBUG')
            for obj in inertial_objects:
                bpy.data.objects.remove(obj)

        linkcount = len(objectlist)
        new_inertial_objects = []
        for obj in objectlist:
            i = 1

            # calculate pose and inertia for the new object
            if self.derive_inertia_from_geometry:
                inertia = inertialib.calculateInertia(obj, self.mass, adjust=True, logging=True)
                pose = obj.matrix_local.to_translation()
            else:
                inertia = [1e-3, 0., 0., 1e-3, 0., 1e-3]
                pose = mathutils.Vector((0.0, 0.0, 0.0))

            # create object from dictionary
            inertialdict = {'mass': self.mass,
                            'inertia': inertia,
                            'pose': {'translation': pose}}
            newinertial = inertialib.createInertial(inertialdict, obj, adjust=True, logging=True)

            if newinertial:
                new_inertial_objects.append(newinertial)

            # update progress bar
            display.setProgress(i / linkcount)
            i += 1

        # select the new inertialobjects
        if new_inertial_objects:
            sUtils.selectObjects(new_inertial_objects, clear=True)

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

    @classmethod
    def poll(cls, context):
        return bpy.ops.object.select_all.poll()

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

    def assignValue(self, name, value):
        prefix = ''
        if isinstance(value, int):
            self.intProp = value
            prefix = 'i'
        elif isinstance(value, str):
            import re

            # make sure eval is called only with true or false
            if re.match('true|false', value[1:], re.IGNORECASE):
                booleanString = value[1:]
                booleanString = booleanString[0].upper() + booleanString[1:].lower()
                self.boolProp = eval(booleanString)
                prefix = 'b'
            else:
                self.stringProp = value
                prefix = 's'
        elif isinstance(value, float):
            self.floatProp = value
            prefix = 'f'
        # TODO what about lists?

        self.name = prefix + '_' + name

    def assignDict(addfunc, dictionary, ignore=[]):
        unsupported = {}
        for propname in dictionary:
            if propname in ignore:
                continue

            # skip subcategories
            if isinstance(dictionary[propname], dict):
                unsupported[propname] = dictionary[propname]
                continue

            subprop = addfunc()
            subprop.assignValue(propname, dictionary[propname])

        return unsupported

    def draw(self, layout, name):
        if self.name[0] == 'i':
            layout.prop(self, 'intProp', text=name)
        elif self.name[0] == 'b':
            layout.prop(self, 'boolProp', text=name)
        elif self.name[0] == 's':
            layout.prop(self, 'stringProp', text=name)
        elif self.name[0] == 'f':
            layout.prop(self, 'floatProp', text=name)


def linkObjectLists(annotation, objectlist):
    """Recursively adds the objects of the specified list to an annotation dictionary.

    Wherever the keyword "$selected_objects:phobostype1:phobostype2" is found as a value in the
    annotation dictionary, the value is replaced by a list of tuples:
        (phobostype, object)
    These tuples represent each an object from the specified objectlist. 'joint' is in this case
    considered a phobostype.

    An arbitrary number of phobostypes can be provided.
    The rest of the annotation dictionary remains untouched.

    Args:
        annotation (dict): annotation dictionary
        objectlist (list(bpy.types.Object)): objects to add to the annotation

    Returns:
        dict -- annotation dictionary with inserted object dictionaries
    """
    newanno = {}
    for key, value in annotation.items():
        if isinstance(value, dict):
            newanno[key] = linkObjectLists(value, objectlist)
        elif "$selected_objects" in value:
            ptypes = value.split(':')[1:]

            objlist = []
            for ptype in ptypes:
                objlist.extend([(ptype, obj) for obj in objectlist if (
                    obj.phobostype == ptype or (obj.phobostype == 'link' and 'joint/type' in obj and
                                                ptype == 'joint'))])
            newanno[key] = objlist
        else:
            newanno[key] = value

    return newanno


def addSensorFromYaml(name, sensortype):
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
        bl_description = 'Add a {0} sensor.'.format(name)
        bl_options = {'INTERNAL'}

        # this contains all the single entries of the dictionary after invoking
        sensor_data = bpy.props.CollectionProperty(type=DynamicProperty)
        annotation_checks = bpy.props.CollectionProperty(type=DynamicProperty)
        sensorName = name
        sensorType = sensortype
        addLink = BoolProperty(default=False)

        def draw(self, context):
            layout = self.layout

            # expose the parameters as the right Property
            for i in range(len(self.sensor_data)):
                name = self.sensor_data[i].name[2:].replace('_', ' ')

                # use the dynamic props name in the GUI, but without the type id
                self.sensor_data[i].draw(layout, name)

            if self.annotation_checks:
                box = layout.box()
                box.label('Include annotations:')
                for i in range(len(self.annotation_checks)):
                    name = self.annotation_checks[i].name[2:].replace('_', ' ')
                    self.annotation_checks[i].draw(box, name)

        def invoke(self, context, event):
            # load the sensor definitions of the current sensor
            data = defs.definitions['sensors'][self.sensorType]

            # ignore properties which should not show up in the GUI
            hidden_props = ['general']

            # identify the property type for all the stuff in the definition
            unsupported = DynamicProperty.assignDict(self.sensor_data.add, data,
                                                     ignore=hidden_props)

            if unsupported:
                for key in unsupported:
                    boolprop = self.annotation_checks.add()
                    boolprop.name = 'b_' + key
                    boolprop.boolProp = False

            # open up the operator GUI
            return context.window_manager.invoke_props_dialog(self)

        def execute(self, context):
            # create a dictionary holding the sensor definition
            sensor_dict = {'name': self.sensorName,
                           'category': defs.def_settings['sensors'][self.sensorType]['categories'],
                           'material': 'phobos_sensor',
                           'type': self.sensorType,
                           'props': {}}
            original_obj = context.active_object
            selected_objs = context.selected_objects

            newlink = None
            if self.addLink:
                newlink = modellinks.createLink({
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
            general_settings = defs.def_settings['sensors'][self.sensorType]
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

            annotation_objs = []
            # add annotation objects for other categories
            for custom_anno in self.annotation_checks:
                if custom_anno.boolProp:
                    # parse object dictionaries if "$selected_objects:..." syntax is found
                    annot = defs.definitions['sensors'][self.sensorType][custom_anno.name[2:]]

                    annot = linkObjectLists(annot, selected_objs)

                    annotation_objs.append(eUtils.addAnnotationObject(
                        sensor_obj, annot,
                        name=nUtils.getObjectName(sensor_obj) + '_' + custom_anno.name[2:],
                        namespace='sensor/' + custom_anno.name[2:]))

            # select the newly added objects
            if newlink:
                sUtils.selectObjects([sensor_obj, newlink] + annotation_objs, clear=True, active=0)
                bUtils.toggleLayer(defs.layerTypes['link'], value=True)
            else:
                sUtils.selectObjects([sensor_obj] + annotation_objs, clear=True, active=0)

            if annotation_objs:
                bUtils.toggleLayer(defs.layerTypes['annotation'], value=True)

            bUtils.toggleLayer(defs.layerTypes['sensor'], value=True)
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
        items = [(sen, sen.replace('_', ' '), '') for sen in sorted(defs.definitions['sensors'])
                 if self.categ in defs.def_settings['sensors'][sen]['categories']]
        return items

    def categorylist(self, context):
        '''Create an enum for the sensor categories. For phobos preset categories,
        the phobosIcon is added to the enum.
        '''
        from phobos.phobosgui import prev_collections

        phobosIcon = prev_collections["phobos"]["phobosIcon"].icon_id
        categories = [t for t in defs.def_subcategories['sensors']]

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
    sensorType = EnumProperty(
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
        layout.prop(self, 'sensorType', text='Sensor type')
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
        if not self.addLink and not context.active_object.phobostype == 'link':
            log('You have no link to add the sensor to. Select one ' +
                'or create it with the operator.', 'INFO')
            return {'CANCELLED'}

        # match the operator to avoid dangers of eval
        import re
        opName = addSensorFromYaml(self.sensorName, self.sensorType)
        operatorPattern = re.compile('[[a-z][a-zA-Z]*\.]*[a-z][a-zA-Z]*')

        # run the operator and pass on add link (to allow undo both new link and sensor)
        if operatorPattern.match(opName):
            eval('bpy.ops.' + opName + "('INVOKE_DEFAULT', addLink=" + str(self.addLink) + ")")
        else:
            log('This sensor name is not following the naming convention: ' + opName +
                '. It can not be converted into an operator.', 'ERROR')
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
    """Add annotations defined by the Phobos definitions """
    bl_idname = "phobos.add_annotations"
    bl_label = "Add Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    def getAnnotationTypes(self, context):
        return [(category,) * 3 for category in sorted(defs.definitions.keys())]

    def getAnnotationCategories(self, context):
        subcategories = [(category,) * 3 for category in
                         sorted(defs.def_subcategories[self.annotationtype])]

        # do not use single categories
        if not subcategories:
            subcategories = [('None',) * 3]
        return subcategories

    def getDeviceTypes(self, context):
        devicetypes = [(device,) * 3 for device in sorted(
            defs.definitions[self.annotationtype].keys()) if self.annotationcategories in
            defs.def_settings[self.annotationtype][device]['categories']]

        if not devicetypes:
            devicetypes = [('None',) * 3]
        return devicetypes

    asObject = BoolProperty(
        name="Add as objects",
        description="Add annotation as object(s)",
        default=True
    )

    annotationtype = EnumProperty(
        items=getAnnotationTypes,
        name="Annotation Type",
        description="Annotation Types")

    annotationcategories = EnumProperty(
        items=getAnnotationCategories,
        name="Categories",
        description="Categories of this annotation type.")

    devicetype = EnumProperty(
        items=getDeviceTypes,
        name="Device Type",
        description="Device Types")

    annotation_data = bpy.props.CollectionProperty(type=DynamicProperty)

    @classmethod
    def poll(cls, context):
        return context is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def check(self, context):
        return True

    def draw(self, context):
        layout = self.layout
        if self.asObject:
            layout.prop(self, 'asObject', text='add annotations as object(s)', icon='FORCE_LENNARDJONES')
        else:
            layout.prop(self, 'asObject', text='add annotations to object', icon='REC')
        layout.separator()

        layout.prop(self, 'annotationtype')

        if self.annotationcategories == 'None':
            layout.label('No categories available.')
        else:
            layout.prop(self, 'annotationcategories')

        # hide devicetype property when empty
        if self.devicetype == 'None':
            layout.label('No devices defined.')
            return

        layout.prop(self, 'devicetype')
        data = defs.definitions[self.annotationtype][self.devicetype]

        hidden_props = ['general']
        # identify the property type for all the stuff in the definition
        self.annotation_data.clear()
        unsupported = DynamicProperty.assignDict(self.annotation_data.add, data,
                                                 ignore=hidden_props)

        # expose the parameters as the right Property
        if self.annotation_data:
            box = layout.box()
            for i in range(len(self.annotation_data)):
                name = self.annotation_data[i].name[2:].replace('_', ' ')

                # use the dynamic props name in the GUI, but without the type id
                self.annotation_data[i].draw(box, name)

            # add unsupported stuff as labels
            for item in unsupported:
                box.label(item, icon='ERROR')

        if unsupported:
            log("These properties are not supported for generic editing: " + str(list(unsupported)),
                'DEBUG')

    def execute(self, context):
        objects = context.selected_objects
        annotation = defs.definitions[self.annotationtype][self.devicetype]

        # add annotation (objects) to the selected objects
        annot_objects = []
        for obj in objects:
            if self.asObject:
                annot_objects.append(eUtils.addAnnotationObject(
                    obj, annotation, name=obj.name + '_annotation',
                    namespace=self.annotationtype.rstrip('s')))
            else:
                eUtils.addAnnotation(obj, annotation, namespace=self.annotationtype.rstrip('s'))

        # reselect the original objects and additional annotation objects
        sUtils.selectObjects(objects + annot_objects, clear=True)
        bUtils.toggleLayer(defs.layerTypes['annotation'], value=True)
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

    @classmethod
    def poll(cls, context):
        # make sure we have a root object with mechanisms
        if not sUtils.getSubmechanismRoots():
            return False
        return True

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout

        layout.prop(self, 'submechanism')

    def execute(self, context):
        root = sUtils.getObjectByProperty('submechanism/name', self.submechanism)
        jointlist = root['submechanism/spanningtree']
        sUtils.selectObjects([root] + jointlist, clear=True, active=0)
        return {'FINISHED'}


class DeleteSubmechanism(Operator):
    """Delete an existing submechanism"""
    bl_idname = "phobos.delete_submechanism"
    bl_label = "Delete Submechanism"
    bl_options = {'REGISTER', 'UNDO'}

    def get_submechanism_roots(self, context):
        return bUtils.compileEnumPropertyList(
            [r['submechanism/name'] for r in sUtils.getSubmechanismRoots()])

    submechanism = EnumProperty(
        name="Submechanism",
        description="submechanism to remove",
        items=get_submechanism_roots
    )

    @classmethod
    def poll(cls, context):
        # make sure we have a root object with mechanisms
        if not sUtils.getSubmechanismRoots():
            return False
        return True

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout

        layout.prop(self, 'submechanism')

    def execute(self, context):
        root = sUtils.getObjectByProperty('submechanism/name', self.submechanism)
        jointlist = root['submechanism/spanningtree']
        objects = [root] + jointlist
        sUtils.selectObjects(objects, clear=True, active=0)

        for obj in objects:
            eUtils.removeProperties(obj, ['submechanism*'])
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
        newroot = context.active_object
        oldroot = sUtils.getRoot(obj=context.active_object)

        # gather model information from old root
        modelprops = eUtils.getProperties(oldroot, category='model')
        oldroot.pose.bones[0].custom_shape = None

        # assign joint resource to oldroot if applicable
        if 'joint/type' in oldroot:
            oldroot.pose.bones[0].custom_shape = ioUtils.getResource(
                ('joint', oldroot['joint/type']))

        eUtils.restructureKinematicTree(newroot)

        # write model information to new root
        newroot.pose.bones[0].custom_shape = ioUtils.getResource(('link', 'root'))
        eUtils.setProperties(newroot, modelprops, category='model')

        # remove model information from old root
        eUtils.removeProperties(oldroot, ['model/*'])
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
        inertials = [obj for obj in context.selected_objects if obj.phobostype == 'inertial']
        self.mass = gUtils.calculateSum(inertials, 'inertial/mass')
        log("The calculated mass is: " + str(self.mass), 'INFO')
        return context.window_manager.invoke_popup(self)

    def execute(self, context):
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
