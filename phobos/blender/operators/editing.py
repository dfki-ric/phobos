#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains all Blender operators for editing of Phobos models.
"""

import math
import os
from datetime import datetime

import bpy
import mathutils
import numpy as np
from bpy.props import (
    BoolProperty,
    IntProperty,
    StringProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    BoolVectorProperty,
    CollectionProperty,
)
from bpy.types import Operator
from idprop.types import IDPropertyGroup
from phobos.io import hyrodyn

from .. import defs as defs
from .. import display as display
from ..io import phobos2blender, blender2phobos
from ..model import controllers as controllermodel
from ..model import inertia as inertialib
from ..model import joints as jUtils
from ..model import links as modellinks
from ..phobosgui import prev_collections
from ..phoboslog import log, ErrorMessageWithBox, WarnMessageWithBox
from ..operators.generic import addObjectFromYaml, DynamicProperty
from ..utils import blender as bUtils
from ..utils import editing as eUtils
from ..utils import general as gUtils
from ..utils import io as ioUtils
from ..utils import naming as nUtils
from ..utils import selection as sUtils
from ..utils import validation as vUtils

from ...io import representation
from ...geometry import io as mesh_io, geometry as geo
from ...utils import resources
from ...io.sensor_representations import Sensor
from ...utils.transform import create_transformation
from ...io import sensor_representations


class SafelyRemoveObjectsFromSceneOperator(Operator):
    """Removes all selected objects from scene, warning if they are deleted"""

    bl_idname = "phobos.safely_remove_objects_from_scene"
    bl_label = "Safely Remove Objects From Scene"
    bl_options = {'UNDO'}

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objs_to_delete = [o for o in context.selected_objects if len(o.users_scene) > 1]
        objs_to_keep = [o for o in context.selected_objects if len(o.users_scene) == 1]
        sUtils.selectObjects(objs_to_delete, clear=True)
        bpy.ops.object.delete()
        sUtils.selectObjects(objs_to_keep, clear=True)

        log(
            "Removed {} object{}".format(
                len(objs_to_delete), 's' if len(objs_to_delete) > 1 else ''
            )
            + " from configuration.",
            'INFO',
        )
        log("    Objects: " + str([obj.name for obj in objs_to_delete]), 'DEBUG')
        if objs_to_keep:
            log('Some objects were not removed as they are unique to this scene.', 'WARNING')
            log("    Objects: " + str([obj.name for obj in objs_to_keep]), 'DEBUG')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return context.selected_objects


class MoveToSceneOperator(Operator):
    """Moves all selected objects to (new) scene"""

    bl_idname = "phobos.move_to_scene"
    bl_label = "Move To Scene"
    bl_options = {'UNDO'}

    def getSceneEnumProperty(self, context):
        """

        Args:
          context:

        Returns:

        """
        scenes = bpy.data.scenes.keys()

        # remove current scene from enum
        scenes.remove(context.scene.name)

        # resources scene is not an export configuration
        if 'resources' in scenes:
            scenes.remove('resources')
        return bUtils.compileEnumPropertyList(scenes)

    scenename : StringProperty(
        name='Scene Name', default='new', description='Name of the scene to which to add selection'
    )

    scene : EnumProperty(
        name='Scene', items=getSceneEnumProperty, description='List of available scenes'
    )

    new : BoolProperty(name='New', default=True, description="Create new scene for configuration")

    truecopy : BoolProperty(
        name='Copy selected', default=False, description="Copy selected objects from active scene"
    )

    link_obdata : BoolProperty(
        name='Link mesh data', default=True, description="Link mesh data to new objects"
    )

    link_material : BoolProperty(
        name='Link material data', default=True, description="Link material data to new objects"
    )

    link_texture : BoolProperty(
        name='Link texture data', default=True, description="Link texture data to new objects"
    )

    remove : BoolProperty(
        name='Remove selected',
        default=False,
        description='Remove selected objects from active scene',
    )

    init : BoolProperty(
        name='Link current scene',
        default=True,
        description="Link all unselected objects from the current scene",
    )

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self, width=500)

    def check(self, context):
        """

        Args:
          context:

        Returns:

        """
        if not self.new:
            self.scenename = self.scene
            self.init = False
        return True

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        """

        Args:
          context:

        Returns:

        """
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

                if obj.phobostype + '/name' not in obj:
                    obj[obj.phobostype + '/name'] = name

            # make object single user
            sUtils.selectObjects(moveobjs, clear=True)
            bpy.ops.object.make_single_user(
                type='SELECTED_OBJECTS',
                object=True,
                obdata=not self.link_obdata,
                material=not self.link_material,
                texture=not self.link_texture,
            )
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

        log(
            "{} {} object{} to {}{}export configuration '{}'.".format(
                'Added',
                len(moveobjs),
                's' if len(moveobjs) > 1 else '',
                'new ' if self.new else '',
                'initialized ' if self.init else '',
                self.scenename,
            ),
            'INFO',
        )
        log("  Objects: " + str([obj.name for obj in moveobjs]), 'DEBUG')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) > 0


class SortObjectsToLayersOperator(Operator):
    """Sort all selected objects to their according layers"""

    bl_idname = "phobos.sort_objects_to_layers"
    bl_label = "Sort Objects to Layers"
    bl_options = {'UNDO'}

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        eUtils.sortObjectsToLayers(context.selected_objects)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) > 0

# [TODO v2.1.0] REFACTOR THIS
class AddKinematicChainOperator(Operator):
    """Add a kinematic chain between two selected objects"""

    bl_idname = "phobos.add_kinematic_chain"
    bl_label = "Add Kinematic Chain"
    bl_options = {'REGISTER', 'UNDO'}

    chainname : StringProperty(
        name='Chain Name', default='new_chain', description='Name of the chain to be created'
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) == 2


class SetXRayOperator(Operator):
    """Show the selected/chosen objects via X-ray"""

    bl_idname = "phobos.set_xray"
    bl_label = "X-Ray Vision"
    bl_options = {'REGISTER', 'UNDO'}

    objects : EnumProperty(
        name="Objects",
        default='selected',
        items=(('all',) * 3, ('selected',) * 3, ('by name',) * 3) + defs.phobostypes,
        description="Show objects via x-ray",
    )

    show : BoolProperty(name="Enable X-Ray", default=True, description="Enable or disable X-Ray")

    namepart : StringProperty(
        name="Name Contains",
        default="",
        description="Part of a name for objects to be selected in 'by name' mode",
    )

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return context.mode == 'OBJECT' or context.mode == 'POSE'

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.label(text="Select items for X-ray view")

        layout.prop(self, "objects")
        layout.prop(self, "show")

        # show name text field only when changing by name
        if self.objects == 'by name':
            layout.prop(self, "namepart")

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
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

    phobostype : EnumProperty(
        items=defs.phobostypes, name="Phobostype", default="undefined", description="Phobostype"
    )

    def execute(self, context):
        """Change phobostype of all selected objects.

        Args:
          context:

        Returns:

        """
        if self.phobostype == 'undefined':
            log("Setting phobostype 'undefined' for selected objects.", 'WARNING')

        for obj in context.selected_objects:
            obj.phobostype = self.phobostype
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return context.selected_objects

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        if context.object:
            self.phobostype = context.object.phobostype
        return context.window_manager.invoke_props_dialog(self)


class BatchEditPropertyOperator(Operator):
    """Edit custom property of selected object(s)"""

    bl_idname = "phobos.batch_property"
    bl_label = "Edit Custom Property"
    bl_options = {'REGISTER', 'UNDO'}

    property_name : StringProperty(name="Name", default="", description="Custom property name")

    property_value : StringProperty(name="Value", default="", description="Custom property value")

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        return ob is not None and len(context.selected_objects) > 0 and ob.mode == 'OBJECT'


class CreateInterfaceOperator(Operator):
    """Create interface and optionally attach to parent"""

    bl_idname = "phobos.create_interface"
    bl_label = "Create Interface"
    bl_options = {'REGISTER', 'UNDO'}

    interface_name : StringProperty(name='name', default='interface')

    interface_type : StringProperty(name='type', default='default')

    interface_direction : EnumProperty(
        name='direction',
        items=bUtils.compileEnumPropertyList(('outgoing', 'incoming', 'bidirectional')),
        default='outgoing',
    )

    all_selected : BoolProperty(name='all selected', default=False)

    scale : FloatProperty(name='scale', default=1.0)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        if self.all_selected:
            for link in [obj for obj in context.selected_objects]:
                if link.phobostype != "link":
                    link = sUtils.getEffectiveParent(link)
                phobos2blender.createInterface(
                    representation.Interface(
                        name=self.interface_name,
                        parent=link.name,
                        type=self.interface_type,
                        direction=self.interface_direction
                    ),
                    None,
                    self.scale
                )
        else:
            link = context.object
            if link.phobostype != "link":
                link = sUtils.getEffectiveParent(context.object)
            phobos2blender.createInterface(
                representation.Interface(
                        name=self.interface_name,
                        parent=link.name,
                        type=self.interface_type,
                        direction=self.interface_direction
                    ),
                None,
                self.scale
            )
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        if context.object:
            for ob in [context.object, sUtils.getEffectiveParent(context.object)]:
                if ob is not None and ob.mode == 'OBJECT' and hasattr(ob, "phobostype") and ob.phobostype == "link":
                    return True
            return False
        else:
            return True


class CopyCustomProperties(Operator):
    """Copy custom properties of selected object(s)"""

    bl_idname = "phobos.copy_props"
    bl_label = "Copy Custom Properties"
    bl_options = {'REGISTER', 'UNDO'}

    empty_properties : BoolProperty(name='empty', default=False, description="empty properties?")

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        slaves = context.selected_objects
        master = context.active_object
        slaves.remove(master)
        master_props = bUtils.cleanObjectProperties(dict(master.items()), phobostype=master.phobostype)
        for obj in slaves:
            props = bUtils.cleanObjectProperties(master_props, phobostype=obj.phobostype)
            if self.empty_properties:
                for key in obj.keys():
                    del (obj[key])
            for key in props.keys():
                obj[key] = props[key]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        obs = context.selected_objects
        ob = context.active_object
        return len(obs) > 0 and ob is not None and ob.mode == 'OBJECT'


class RenameCustomProperty(Operator):
    """Rename custom property of selected object(s)"""

    bl_idname = "phobos.rename_custom_property"
    bl_label = "Rename Custom Property"
    bl_options = {'REGISTER', 'UNDO'}

    find : StringProperty(
        name="Find Property Name", default='', description="Name to be searched for"
    )

    replace : StringProperty(
        name="Replacement Name", default='', description="New name to be replaced with"
    )

    overwrite : BoolProperty(
        name='Overwrite Existing Properties',
        default=False,
        description="If a property of the specified replacement name exists, overwrite it?",
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objs = filter(lambda e: self.find in e, context.selected_objects)
        if self.replace != "":
            for obj in objs:
                if self.replace in obj and not self.overwrite:
                    log(
                        "Property '"
                        + self.replace
                        + "' already present in"
                        + "object '"
                        + obj.name
                        + "'",
                        "ERROR",
                    )
                else:
                    obj[self.replace] = obj[self.find]
                    del obj[self.find]
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class SetGeometryType(Operator):
    """Edit geometry type of selected object(s)"""

    bl_idname = "phobos.define_geometry"
    bl_label = "Define Geometry"
    bl_options = {'UNDO'}

    geomType : EnumProperty(
        items=defs.geometrytypes, name="Type", default="box", description="Phobos geometry type"
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objs = context.selected_objects
        for obj in objs:
            if obj.phobostype == 'collision' or obj.phobostype == 'visual':
                obj['geometry/type'] = self.geomType
            else:
                log("The object '" + obj.name + "' is no collision or visual.", 'WARNING')

        log(
            "Changed geometry type for {} object{}".format(len(objs), 's' if len(objs) > 1 else '')
            + " to {}.".format(self.geomType),
            'INFO',
        )
        log("    Objects: " + str([obj.name for obj in objs]), 'DEBUG')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout

        layout.prop(self, 'geomType')

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self, width=200)


class SmoothenSurfaceOperator(Operator):
    """Smoothen surface of selected objects"""

    bl_idname = "phobos.smoothen_surface"
    bl_label = "Smoothen Surface"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objs = [obj for obj in context.selected_objects if obj.type == "MESH"]
        i = 1
        for obj in objs:
            eUtils.smoothen_surface(obj)
            display.setProgress(i / len(context.selected_objects))
            i += 1
        display.endProgress()
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT' and len(context.selected_objects) > 0


class EditInertialData(Operator):
    """Edit mass/inertia of selected object(s)"""

    bl_idname = "phobos.edit_inertial_data"
    bl_label = "Edit Mass/Inertia"
    bl_options = {'UNDO'}

    changeInertia : BoolProperty(
        name="Change inertia", default=True, description="Change inertia values"
    )

    inertiavector : FloatVectorProperty(
        name="Inertia Vector",
        default=[1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3],
        subtype='NONE',
        precision=10,
        size=6,
        description="New inertia values for the inertial objects",
    )

    changeMass : BoolProperty(name="Change mass", default=True, description="Change mass values")

    mass : FloatProperty(
        name='Mass',
        default=1e-3,
        description="New mass for the inertial objects (in kg)",
        precision=10,
    )

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        # read initial parameter values from active object
        if context.active_object and context.active_object.phobostype == 'inertial':
            errors, *_ = vUtils.validateInertiaData(context.active_object, adjust=True)
            for error in errors:
                error.log()

            self.mass = context.active_object['mass']
            self.inertiavector = mathutils.Vector(context.active_object['inertia'])

        return context.window_manager.invoke_props_dialog(self, width=200)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
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
                obj['mass'] = newmass
            if self.changeInertia:
                obj['inertia'] = newinertia

        if self.changeMass:
            log("Changed mass to " + str(newmass) + " for {} objects.".format(len(objs)), 'INFO')
        if self.changeInertia:
            log(
                "Changed inertia to " + str(newinertia) + " for {} objects.".format(len(objs)),
                'INFO',
            )
        return {'FINISHED'}

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        """

        Args:
          context:

        Returns:

        """
        return True

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return bool([obj for obj in context.selected_objects if obj.phobostype == 'inertial'])


class GenerateInertialObjectsOperator(Operator):
    """Creates inertial object(s) to add mass and inertia data to links"""

    bl_idname = "phobos.generate_inertial_objects"
    bl_label = "Create Inertials"
    bl_options = {'REGISTER', 'UNDO'}

    mass : FloatProperty(name='Mass', default=0.001, description="Mass (of active object) in kg")

    derive_inertia_from_geometry : BoolProperty(
        name="Calculate inertia from geometry ",
        default=True,
        description="Derive inertia value(s) from geometry of visual or collision objects.",
    )

    clear : BoolProperty(
        name="Clear existing inertial objects",
        default=True,
        description="Clear existing inertial objects of selected links.",
    )

    def toggleVisual(self, context):
        self.collisions = not bool(self.visuals)

    def toggleCollision(self, context):
        self.visuals = not bool(self.collisions)

    visuals : BoolProperty(
        name="visual",
        default=True,
        description="Use the selected visual objects for inertial creation.",
        update=toggleVisual
    )

    collisions : BoolProperty(
        name="collision",
        default=False,
        description="Use the selected visual objects for inertial creation.",
        update=toggleCollision
    )

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        geometric_objects = [
            obj for obj in context.selected_objects if obj.phobostype in ['visual', 'collision']
        ]
        visuals = [obj for obj in geometric_objects if obj.phobostype == 'visual']
        collisions = [obj for obj in geometric_objects if obj.phobostype == 'collision']

        if not visuals:
            self.collisions = True
            self.visuals = False
        if not collisions:
            self.visuals = True
            self.collisions = False

        # initialise the geometry parameter correctly
        if not geometric_objects:
            self.derive_inertia_from_geometry = False
        else:
            self.derive_inertia_from_geometry = True

        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.prop(self, 'clear')
        geometric_objects = [
            obj for obj in context.selected_objects if obj.phobostype in ['visual', 'collision']
        ]

        if geometric_objects:
            layout.prop(self, 'derive_inertia_from_geometry')
            visuals = [obj for obj in geometric_objects if obj.phobostype == 'visual']
            collisions = [obj for obj in geometric_objects if obj.phobostype == 'collision']

            layout.label(text="Use geometry objects:")
            row = layout.row(align=True)
            if not visuals:
                row.label(text='visuals')
            else:
                row.prop(self, 'visuals', toggle=True)
            if not collisions:
                row.label(text='collisions')
            else:
                row.prop(self, 'collisions', toggle=True)

        layout.prop(self, 'mass')

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        # store previously selected objects
        selection = context.selected_objects
        viscols = [obj for obj in selection if obj.phobostype in ['visual', 'collision']]
        if any([obj.parent is None for obj in viscols]):
            ErrorMessageWithBox(f"You have selected objects that don't have a parent, those will be ignored", reporter=self)
            viscols = [obj for obj in selection if obj.phobostype in ['visual', 'collision'] if obj.parent is not None]
        links = list(
            set(
                [obj.parent for obj in viscols if obj.parent is not None]
                + [obj for obj in selection if obj.phobostype == 'link']
            )
        )

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
            inertial_objects = [
                obj for link in links for obj in link.children if obj.phobostype == 'inertial'
            ]
            log("Removing old inertial objects: " + str(inertial_objects), 'DEBUG')
            for obj in inertial_objects:
                bpy.data.objects.remove(obj)

        linkcount = len(objectlist)
        new_inertial_objects = []
        for obj in objectlist:
            i = 1
            mass = self.mass
            # calculate pose and inertia for the new object
            if self.derive_inertia_from_geometry:
                if "mass" in obj:
                    mass = obj["mass"]
                geometry = blender2phobos.deriveGeometry(obj)
                inertia = inertialib.calculateInertia(obj, mass, geometry, adjust=True, logging=True)
                if isinstance(geometry, representation.Mesh):
                    _, pose = geometry.approx_volume_and_com()
                    pose = np.array(obj.matrix_local).dot(create_transformation(xyz=pose))[0:3, 3]
                else:
                    pose = obj.matrix_local.to_translation()
            else:
                inertia = [1e-3, 0., 0., 1e-3, 0., 1e-3]
                pose = mathutils.Vector((0.0, 0.0, 0.0))

            # create object from dictionary
            if not sUtils.getEffectiveParent(obj, include_hidden=True, ignore_selection=True):
                ErrorMessageWithBox(f"{obj.name} has no parent link to which the inertial could be added", reporter=self)
                continue
            inertial = representation.Inertial(
                mass=mass,
                inertia=representation.Inertia(*inertia),
                origin=representation.Pose(xyz=pose, relative_to=sUtils.getEffectiveParent(obj, ignore_selection=True, include_hidden=True).name)
            )
            newinertial = phobos2blender.createInertial(inertial, sUtils.getEffectiveParent(obj, ignore_selection=True, include_hidden=True), adjust=True, logging=True)

            if newinertial:
                new_inertial_objects.append(newinertial)

            # update progress bar
            display.setProgress(i / linkcount)
            i += 1
        display.endProgress()
        # select the new inertialobjects
        if new_inertial_objects:
            sUtils.selectObjects(new_inertial_objects, clear=True)

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return context.selected_objects is not None and bool(
            {'visual', 'collision', 'inertial', 'link'}
            & set([o.phobostype for o in context.selected_objects])
        )


class CreateCollisionObjects(Operator):
    """Create collision objects for all selected visual objects"""

    bl_idname = "phobos.create_collision_objects"
    bl_label = "Create Collision Object(s)"
    bl_options = {'REGISTER', 'UNDO'}

    # [TODO v2.1.0] Add convex
    property_colltype : EnumProperty(
        name='Collision Type', default='box', description="Collision type", items=defs.geometrytypes
    )

    # [TODO v2.1.0] Fix optimized creation see: Fix creation for Trimesh in geometry/geometry.py
    # property_optimized : BoolProperty(
    #     name='Optimize', default=False, description="Whether you want to add an improved sized and oriented primitive"
    # )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        visuals = []
        collisions = []

        # find all selected visual objects
        for obj in context.selected_objects:
            if obj.phobostype == "visual":
                visuals.append(obj)
            obj.select_set(False)

        if not visuals:
            log("No visual objects selected.", "ERROR", self)
            return {'CANCELLED'}

        # create collision objects for each visual
        for vis in visuals:
            # build object names
            if "visual" in vis.name.lower():
                collname = vis.name.replace("visual", "collision").replace("Visual", "Collision").replace("VISUAL", "COLLISION")
            else:
                collname = vis.name + "_collision"

            materialname = vis.data.materials[0].name if len(vis.data.materials) > 0 else "None"

            phobos_vis = blender2phobos.deriveVisual(vis)

            # create Mesh
            if self.property_colltype != 'mesh':
                geometry = None
                transform = np.identity(4)
                if self.property_colltype == "box":
                    # [TODO v2.1.0] Fix optimized creation see: Fix creation for Trimesh in geometry/geometry.py
                    # geometry, transform = geo.create_box(
                    #     vis if not self.property_optimized else mesh_io.as_trimesh(vis.data),
                    #     scale=getattr(phobos_vis, "scale", 1), oriented=self.property_optimized
                    # )
                    # geometry, transform = geo.create_box(vis, scale=getattr(phobos_vis, "scale", 1), oriented=self.property_optimized)
                    geometry, transform = geo.create_box(vis, scale=getattr(phobos_vis, "scale", 1), oriented=False)
                elif self.property_colltype == "cylinder":
                    # [TODO v2.1.0] Fix optimized creation see: Fix creation for Trimesh in geometry/geometry.py
                    # geometry, transform = geo.create_cylinder(
                    #     vis if not self.property_optimized else mesh_io.as_trimesh(vis.data),
                    #     scale=getattr(phobos_vis, "scale", 1),
                    # )
                    geometry, transform = geo.create_cylinder(vis, scale=getattr(phobos_vis, "scale", 1),)
                elif self.property_colltype == "sphere":
                    # [TODO v2.1.0] Fix optimized creation see: Fix creation for Trimesh in geometry/geometry.py
                    # geometry, transform = geo.create_sphere(
                    #     vis if not self.property_optimized else mesh_io.as_trimesh(vis.data),
                    #     scale=getattr(phobos_vis, "scale", 1),
                    # )
                    geometry, transform = geo.create_sphere(vis, scale=getattr(phobos_vis, "scale", 1),)
                elif self.property_colltype == "convex":
                    geometry = blender2phobos.deriveGeometry(vis, duplicate_mesh=True)
                    geometry.to_convex_hull()
                    geometry.apply_scale()
                link = sUtils.getEffectiveParent(vis, include_hidden=True, ignore_selection=True)
                if link is None:
                    ErrorMessageWithBox(message="Before creating collision parent visual to a link")
                    return {'CANCELLED'}
                collision = representation.Collision(
                    name=collname,
                    link=link.name,
                    geometry=geometry,
                    origin=representation.Pose.from_matrix(phobos_vis.origin.to_matrix().dot(transform), relative_to=link.name)
                )
                ob = phobos2blender.createGeometry(collision, geomsrc="collision", linkobj=sUtils.getEffectiveParent(vis, include_hidden=True, ignore_selection=True))
            else:
                ob = bUtils.createPrimitive(
                    collname,
                    'cylinder',
                    (1., 1., 1.),
                    defs.layerTypes['collision'],
                    materialname,
                    phobos_vis.origin.position,
                    phobos_vis.origin.rotation,
                    'collision'
                )
                ob.scale = vis.scale
                ob.data = vis.data  # we don't do vis.data.copy() to have a multi-user mesh

            # set properties of new collision object
            ob.phobostype = 'collision'
            ob['geometry/type'] = self.property_colltype
            collisions.append(ob)

            # make collision object relative if visual object has a parent
            if vis.parent:
                # [ToDo v2.1.0] REVIEW: removal of this should be correct, please evaluate and remove
                # ob.select_set(True)
                # bpy.ops.object.transform_apply(location=False, rotation=False, scale=True, properties=False)
                # vis.parent.select_set(True)
                eUtils.parentObjectsTo(context.selected_objects, vis.parent)

            # select created collision objects
            sUtils.selectObjects(collisions)
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) > 0


class SetCollisionGroupOperator(Operator):
    """Set the collision groups of the selected collision object(s)"""

    bl_idname = "phobos.set_collision_group"
    bl_label = "Set Collision Group(s)"
    bl_options = {'REGISTER', 'UNDO'}

    groups : BoolVectorProperty(
        name='Collision Groups',
        size=20,
        subtype='LAYER',
        default=(False,) * 20,
        description='Collision groups',
    )

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        try:
            self.groups = context.active_object.rigid_body.collision_groups
        # create rigid body settings if not existent in active object
        except AttributeError:
            obj = context.active_object
            bpy.ops.rigidbody.object_add(type='ACTIVE')
            obj.rigid_body.kinematic = True
            obj.rigid_body.collision_groups = self.groups
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objs = filter(
            lambda e: "phobostype" in e and e.phobostype == "collision", context.selected_objects
        )
        active_object = context.active_object

        # try assigning the collision groups to each selected collision object
        for obj in objs:
            try:
                obj.rigid_body.collision_groups = self.groups
            # initialize rigid body settings if necessary
            except AttributeError:
                context.view_layer.objects.active = obj
                bpy.ops.rigidbody.object_add(type='ACTIVE')
                obj.rigid_body.kinematic = True
                obj.rigid_body.collision_groups = self.groups
        context.view_layer.objects.active = active_object
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        return ob is not None and ob.phobostype == 'collision' and ob.mode == 'OBJECT'


class DefineJointConstraintsOperator(Operator):
    """Add bone constraints to the joint (link)"""

    bl_idname = "phobos.define_joint_constraints"
    bl_label = "Define Joint(s)"
    bl_options = {'REGISTER', 'UNDO'}

    name : StringProperty(
        name='Joint Name (leave empty for same name as link)', default="", description='Defines the name of the joint'
    )

    active : BoolProperty(
        name='Active', default=False, description='Add an motor to the joint'
    )

    useRadian : BoolProperty(
        name='Use Radian', default=True, description='Use degrees or rad for joints'
    )

    joint_type : EnumProperty(
        name='Joint Type',
        default='revolute',
        description="Type of the joint",
        items=defs.jointtypes,
    )

    lower : FloatProperty(name="Lower", default=-3.14, description="Lower constraint of the joint")

    upper : FloatProperty(name="Upper", default=3.14, description="Upper constraint of the joint")

    maxeffort : FloatProperty(
       name="Max Effort (N or Nm)", default=0.0, description="Maximum effort of the joint"
    )

    maxvelocity : FloatProperty(
       name="Max Velocity (m/s or rad/s)",
       default=0.0,
       description="Maximum velocity of the joint. If you uncheck radian, you can enter °/sec here",
    )

    spring : FloatProperty(
        name="Spring Constant", default=0.0, description="Spring constant of the joint"
    )

    damping : FloatProperty(
        name="Damping Constant", default=0.0, description="Damping constant of the joint"
    )

    axis: FloatVectorProperty(
        name="Joint Axis", default=[0.0, 0.0, 1], description="Damping constant of the joint", size=3
    )

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        if len(context.selected_objects) == 1:
            layout.prop(self, "name")
        layout.prop(self, "joint_type", text="joint_type")

        # enable/disable optional parameters
        if not self.joint_type == 'fixed':
            layout.prop(self, "active", text="Active (adds a default motor you can adapt later)")
            if self.joint_type in ["revolute", "prismatic", "continuous"]:
                layout.prop(self, "axis", text="Sets the joint axis")
            if self.joint_type == "revolute":
                layout.prop(self, "useRadian", text="use radian")
            layout.prop(
                self,
                "maxeffort",
                text="max effort ["
                + ('Nm]' if self.joint_type in ['revolute', 'continuous'] else 'N]'),
            )
            if self.joint_type in ['revolute', 'continuous']:
                layout.prop(
                    self,
                    "maxvelocity",
                    text="max velocity [" + ("rad/s]" if self.useRadian else "°/s]"),
                )
            else:
                layout.prop(self, "maxvelocity", text="max velocity [m/s]")
            if self.joint_type == 'revolute':
                layout.prop(self, "lower", text="lower [rad]" if self.useRadian else "lower [°]")
                layout.prop(self, "upper", text="upper [rad]" if self.useRadian else "upper [°]")
                layout.prop(self, "spring", text="spring constant [N/m]")
                layout.prop(self, "damping", text="damping constant")
            elif self.joint_type == 'prismatic':
                layout.prop(self, "lower", text="lower [m]")
                layout.prop(self, "upper", text="upper [m]")
                layout.prop(self, "spring", text="spring constant [N/m]")
                layout.prop(self, "damping", text="damping constant")

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        aObject = context.active_object
        #if 'joint/type' not in aObject and 'motor/type' in aObject:
        #    self.maxvelocity = aObject['motor/maxSpeed']
        #    self.maxeffort = aObject['motor/maxEffort']
        return self.execute(context)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        log('Defining joint constraints for joint: ', 'INFO')
        lower = 0
        upper = 0
        velocity = self.maxvelocity
        effort = self.maxeffort

        # lower and upper limits
        if self.joint_type in ["revolute", "continuous", "sphere"]:
            # velocity calculation
            if not self.useRadian:
                velocity = self.maxvelocity * ((2 * math.pi) / 360)  # from °/s to rad/s
            else:
                velocity = self.maxvelocity
        if self.joint_type == 'revolute':
            if not self.useRadian:
                lower = math.radians(self.lower)
                upper = math.radians(self.upper)
            else:
                lower = self.lower
                upper = self.upper
        elif self.joint_type == "prismatic":
            lower = self.lower
            upper = self.upper
        axis = None
        if self.joint_type in ["revolute", "prismatic", "continuous"]:
            axis = self.axis
        # set properties for each joint
        for joint in (obj for obj in context.selected_objects if obj.phobostype == 'link'):
            context.view_layer.objects.active = joint
            assert joint.parent is not None and joint.parent.phobostype == "link", \
                f"You need to have a link parented to {joint.name} before you can create a joint"
            if len(self.name) > 0:
                joint["joint/name"] = self.name
            jUtils.setJointConstraints(
                joint=joint,
                jointtype=self.joint_type,
                lower=lower,
                upper=upper,
                velocity=velocity,
                effort=effort,
                spring=self.spring,
                damping=self.damping,
                axis=(np.array(axis) / np.linalg.norm(axis)).tolist() if axis is not None else None
            )

            if "joint/name" not in joint:
                joint["joint/name"] = joint.name + "_joint"

            motor_name = joint.get("joint/name", joint.name) + "_motor"
            phobos2blender.createMotor(
                motor=representation.Motor(
                    name=motor_name,
                    joint=joint.get("joint/name", joint.name),
                    **resources.get_default_motor()
                ),
                linkobj=joint
            )

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        # due to invoke the active object needs to be a link
        return ob is not None and ob.phobostype == 'link' and ob.mode == 'OBJECT'


class DissolveLink(Operator):
    """Dissolve a link and parent all children of type link, inertia, visual and collision to its effective parent"""

    bl_idname = "phobos.dissolve_link"
    bl_label = "Dissolve Link(s)"
    bl_options = {"UNDO"}

    delete : BoolProperty(
        name='Delete Other', default=False, description='Delete all non reparented children'
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objects = context.selected_objects

        while objects:
            # Get the roots
            current_roots = sUtils.getLeaves(objects, objects=objects)
            for root in current_roots:
                eUtils.dissolveLink(root, delete_other=self.delete)
                objects.remove(root)
        return {'FINISHED'}

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.prop(self, 'delete')

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self)

    def check(self, context):
        """

        Args:
          context:

        Returns:

        """
        return True

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        objs = list(
            filter(lambda e: "phobostype" in e and e.phobostype == "link", context.selected_objects)
        )
        return ob is not None and ob.phobostype == 'link' and context.mode == "OBJECT"


class AddMotorOperator(Operator):
    """Add a motor to the selected joint.
    It is possible to add motors to multiple joints at the same time.

    Args:

    Returns:

    """

    bl_idname = "phobos.add_motor"
    bl_label = "Add Motor"
    bl_options = {'UNDO'}
    lastMotorDefault = None

    template : EnumProperty(items=resources.get_motor_defaults(), description="The template to use for this motor")
    motorType : EnumProperty(items=representation.Motor.BUILD_TYPES, description='The motor type')
    controllerType: EnumProperty(items=representation.Motor.TYPES, description='The controller type')
    maxeffort : FloatProperty(
        name="Max Effort (N or Nm)", default=0.0, description="Maximum effort of the joint"
    )

    maxvelocity : FloatProperty(
        name="Max Speed (m/s or rad/s)",
        default=0.0,
        description="Maximum velocity of the joint. If you uncheck radian, you can enter °/sec here",
    )
    controlp : FloatProperty(
        name="P Gain", default=0.0, description="P gain of position controller."
    )
    controli : FloatProperty(
        name="I Factor", default=0.0, description="Integral factor of position controller."
    )
    controld : FloatProperty(
        name="D Factor", default=0.0, description="D factor of position controller"
    )
    knownProperties = {"effort": "maxeffort",
                       "velocity": "maxvelocity",
                       "p": "controlp",
                       "i": "controli",
                       "d": "controld"}

    def updateValues(self, key, defDict, lastDict, prop):
        # only update value if the user hasn't change the default value
        if key in defDict:
            if lastDict and key in lastDict and prop != lastDict[key]:
                return prop
            return defDict[key]
        return prop

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        #layout.separator()
        setvalues = self.lastMotorDefault != self.template
        lastDict = None
        if self.lastMotorDefault != None:
            lastDict = resources.get_default_motor(self.lastMotorDefault)
        defDict = resources.get_default_motor(self.template)
        layout.prop(self, 'template', text='Motor template')
        layout.label(text="Parameters:")
        for k, v in self.knownProperties.items():
            if setvalues:
                setattr(self, v, self.updateValues(k, defDict, lastDict, getattr(self, v)))
            if k in defDict:
                layout.prop(self, v)
        self.lastMotorDefault = self.template

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self)

    def check(self, context):
        """

        Args:
          context:

        Returns:

        """
        return True

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        active_obj = (
            context.active_object
            and context.active_object.phobostype == 'link'
            and context.active_object.mode == 'OBJECT'
        )

        if not active_obj:
            return False
            # for obj in context.selected_objects:
            #     if obj.mode == 'OBJECT' and obj.phobostype == 'link':
            #         active_obj = obj
            #         context.view_layer.objects.active = obj
            #         break
            # if not active_obj:
        else:
            active_obj = context.active_object

        joint_obj = 'joint/type' in active_obj and active_obj['joint/type'] != 'fixed'

        return joint_obj

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        objects = [o for o in context.selected_objects if o.phobostype == "link"]
        for obj in objects:
            phobos2blender.createMotor(representation.Motor(
                name=obj.name+"_motor",
                joint=obj.get("joint/name", obj.name),
                type=self.controllerType,
                build_type=self.motorType,
                p=self.controlp,
                i=self.controli,
                d=self.controld
            ), linkobj=obj)
        return {'FINISHED'}


# [TODO v2.1.0] Can we delete this
# def addMotorFromYaml(motor_dict, annotations, selected_objs, active_obj, *args):
#     """Execution function for the temporary operator to add motors from yaml files.
#
#     The specified parameters match the interface of the `addObjectFromYaml` generic function.
#
#     As additional argument, a boolean value is required. It controls whether the specified motor
#     will be added to all selected joints (True) or to the active object only (False).
#
#     Args:
#       motor_dict(dict): phobos representation of a motor
#       annotations(dict): annotation dictionary containing annotation categories as keys
#       selected_objs(list(bpy.types.Object): selected objects in the current context
#       active_obj(bpy.types.Object): active object in the current context
#       *args(list): list containing a single bool value
#
#     Returns:
#       tuple: lists of new motor, new annotation and controller objects
#
#     """
#     addtoall = args[0]
#     addcontrollers = args[1]
#
#     if addtoall:
#         joints = [lnk for lnk in selected_objs if lnk.phobostype == 'link' and 'joint/type' in lnk]
#     else:
#         joints = [active_obj]
#
#     newmotors = []
#     annotation_objs = []
#     controller_objs = []
#     for joint in joints:
#         pos_matrix = joint.matrix_world
#         motor_dict['name'] = ''
#         motor_obj = phobos2blender.createMotor(
#             motor_dict, joint, pos_matrix, addcontrollers=addcontrollers
#         )
#
#         if isinstance(motor_obj, list):
#             controller_objs.append(motor_obj[1])
#             motor_obj = motor_obj[0]
#
#         # parent motor to its joint
#         sUtils.selectObjects([motor_obj, joint], clear=True, active=1)
#         bpy.ops.object.parent_set(type='BONE_RELATIVE')
#
#         newmotors.append(motor_obj)
#
#         # add optional annotation objects
#         for annot in annotations:
#             annotation_objs.append(
#                 eUtils.addAnnotationObject(
#                     motor_obj,
#                     annotations[annot],
#                     name=nUtils.getObjectName(motor_obj) + '_' + annot,
#                     namespace='motor/' + annot,
#                 )
#             )
#     return newmotors, annotation_objs, controller_objs


class CreateLinksOperator(Operator):
    """Create link(s), optionally based on existing objects"""

    bl_idname = "phobos.create_links"
    bl_label = "Create Link(s)"
    bl_options = {'REGISTER', 'UNDO'}

    location : EnumProperty(
        items=(('3D cursor',) * 3, ('selected objects',) * 3),
        default='3D cursor',
        name='Location',
        description='Where to create new link(s)?',
    )

    size : FloatProperty(name="Visual Size", default=1.0, description="Size of the created link")

    parent_link : BoolProperty(
        name="Parent Link", default=False, description="Parent link to object's parents"
    )

    parent_objects : BoolProperty(
        name='Parent Objects', default=False, description='Parent children of object to new link'
    )

    nameformat : StringProperty(
        name="Name Format",
        description="Provide a string containing {0} {1} etc. to reuse parts of objects' names.",
        default='',
    )

    linkname : StringProperty(
        name="Link Name", description="A name for a single newly created link.", default='new_link'
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        if self.location == '3D cursor':
            phobos2blender.createLink(representation.Link(name=self.linkname))
        elif len(context.selected_objects) > 0:
            objs_to_create_links = context.selected_objects
            for obj in objs_to_create_links:
                modellinks.deriveLinkfromObject(obj)
        else:
            WarnMessageWithBox("No objects selected to create links from!")
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return bpy.ops.object.select_all.poll()

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.prop(self, "location")
        layout.prop(self, "size")
        if self.location == '3D cursor':
            layout.prop(self, 'linkname')
        else:
            layout.prop(self, "nameformat")
            layout.prop(self, "parent_link")
            layout.prop(self, "parent_objects")


class AddSensorOperator(Operator):
    """Add a sensor at the position of the selected object.
    It is possible to create a new link for the sensor on the fly. Otherwise,
    the next link in the hierarchy will be used to parent the sensor to.

    Args:

    Returns:

    """

    bl_idname = "phobos.add_sensor"
    bl_label = "Add Sensor"
    bl_options = {'REGISTER', 'UNDO'}


    def sensorlist(self, context):
        """

        Args:
          context:

        Returns: A list of available sensor categories. Taken from defaults.json
            Format: [('Sensor_name', 'Sensor name', ''),...]

        """

        items = [
            (sen, sen.replace('_', ' '), '')
            for sen in sorted(resources.get_sensor_types(self.category))
        ]
        # Alternative: sensor_representations w/o factory, sensor, multisensor
        return items

    def categorylist(self, context):
        """Create an enum for the sensor categories. For phobos preset categories,
        the phobosIcon is added to the enum.

        Args:
          context:

        Returns:

        """
        categories = resources.get_sensor_categories()

        items = []
        i = 0
        for categ in categories:
            # assign an icon to the phobos preset categories
            if categ == 'cameraSensor':
                icon = 'CAMERA_DATA'
            else:
                icon = 'LAYER_USED'  # TODO add icons

            items.append((categ, categ, categ, icon, i))
            i += 1

        return items

    category: EnumProperty(items=categorylist, description='The sensor category')
    sensorType: EnumProperty(items=sensorlist, description='The sensor type')
    sensorName: StringProperty(
        name="Sensor name", default='new_sensor', description="Name of the sensor"
    )

    # dynamic properties of the sensor
    sensorProperties: CollectionProperty(type=DynamicProperty)

    currentSensor = ("", "")

    def updateSensorProperties(self):
        """
        Updates the dynamic sensor properties after changing sensor category or type

        Args:

        Returns:

        """
        if self.category != self.currentSensor[0] or self.sensorType != self.currentSensor[1]:
            data = resources.get_sensor(self.category, self.sensorType)
            self.sensorProperties.clear()
            DynamicProperty.assignDict(
                self.sensorProperties.add, data
            )
            for prop in self.sensorProperties:
                prop.allowDisabling()
            self.currentSensor = (self.category, self.sensorType)


    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.prop(self, 'sensorName')
        layout.separator()
        layout.prop(self, 'category', text='Sensor category')
        layout.prop(self, 'sensorType', text='Sensor type')
        layout.separator()

        # Draw sensor properties
        self.updateSensorProperties()
        for i in range(len(self.sensorProperties)):
            name = self.sensorProperties[i].name.replace('_', ' ')

            # use the dynamic props name in the GUI, but without the type id
            self.sensorProperties[i].draw(layout, name)
        layout.label(text="You can add custom properties under")
        layout.label(text="Object Properties > Custom Properties")

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self)

    # def check(self, context):
    #     """
    #
    #     Args:
    #       context:
    #
    #     Returns:
    #
    #     """
    #     return True

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:
            True if there is a link we can attach a new sensor to
            False otherwise

        """
        linkFound, link = cls.getLink(context)
        return linkFound

    def getSensorParameters(self):
        """

        Args:

        Returns: The parameters entered by the user for the selected sensor

        """
        result = DynamicProperty.collectDict(self.sensorProperties)
        return result

    @classmethod
    def getLink(cls, context):
        """

        Args:
          context:

        Returns:
            False, None if neither the selection nor their parent are links
            True, theLink otherwise

        """
        link = context.active_object
        if link is None:
            return False, None
        if not context.active_object.phobostype == 'link':  # Selection is no link, get their parent
            link = sUtils.getEffectiveParent(link)
        if not context.active_object.phobostype == 'link':  # Parent is no link either
            return False, None
        return True, link

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        # make sure a link or its child is selected
        linkFound, link = self.getLink(context)
        if not linkFound:
            return {'CANCELLED'}

        # Create Sensor
        sensorName = self.sensorName
        parameters = self.getSensorParameters()
        # Get sensor category specific class
        sensorClass = getattr(sensor_representations, self.category)
        if "link" in sensorClass._class_variables:
            parameters["link"] = parameters.get("link", link.name)
        if "joint" in sensorClass._class_variables:
            parameters["link"] = parameters.get("joint", link.get("joint/name", link.name))
        if "frame" in sensorClass._class_variables:
            parameters["frame"] = parameters.get("frame", link.name)
        sensor = sensorClass(
            name = sensorName,
            **parameters # Pass sensor specific parameters
        )
        sensor_obj = phobos2blender.createSensor(sensor, linkobj=link)



        # match the operator to avoid dangers of eval
        # import re

        # opName = addObjectFromYaml(
        #     self.sensorName, 'sensor', self.sensorType, addSensorFromYaml, self.addLink
        # )

        # operatorPattern = re.compile('[[a-z][a-zA-Z]*\.]*[a-z][a-zA-Z]*')

        # # run the operator and pass on add link (to allow undo both new link and sensor)
        # if operatorPattern.match(opName):
        #     eval('bpy.ops.' + opName + "('INVOKE_DEFAULT')")
        # else:
        #     log(
        #         'This sensor name is not following the naming convention: '
        #         + opName
        #         + '. It can not be converted into an operator.',
        #         'ERROR',
        #     )
        return {'FINISHED'}

#
# # [TODO v2.0.0] REVIEW this
# def addControllerFromYaml(controller_dict, annotations, selected_objs, active_obj, *args):
#     """Execution function for the temporary operator to add controllers from yaml files.
#
#     The specified parameters match the interface of the `addObjectFromYaml` generic function.
#
#     Args:
#       controller_dict(dict): phobos representation of a controller
#       annotations(dict): annotation dictionary containing annotation categories as keys
#       selected_objs(list(bpy.types.Object): selected objects in the current context
#       active_obj(bpy.types.Object): active object in the current context
#       *args(list): empty list
#
#     Returns:
#       tuple: tuple of lists of new motor, new controller and new annotation objects
#
#     """
#     addtoall = args[0]
#
#     if addtoall:
#         objects = [obj for obj in selected_objs if obj.phobostype in defs.controllabletypes]
#     else:
#         objects = [active_obj]
#
#     controller_objs = []
#     annotation_objs = []
#     for obj in objects:
#         pos_matrix = obj.matrix_world
#         controller_obj = controllermodel.createController(controller_dict, obj, pos_matrix)
#
#         # add optional annotation objects
#         for annot in annotations:
#             annotation_objs.append(
#                 eUtils.addAnnotationObject(
#                     controller_obj,
#                     annotations[annot],
#                     name=nUtils.getObjectName(controller_obj) + '_' + annot,
#                     namespace='controller/' + annot,
#                 )
#             )
#         controller_objs.append(controller_obj)
#
#     return controller_objs, annotation_objs, []
#
#
# # [TODO v2.0.0] REVIEW this
# class AddControllerOperator(Operator):
#     """Add a controller at the position of the selected object."""
#
#     bl_idname = "phobos.add_controller"
#     bl_label = "Add Controller"
#     bl_options = {'UNDO'}
#
#     def controllerlist(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         items = [
#             (con, con.replace('_', ' '), '')
#             for con in sorted(defs.definitions['controllers'])
#             if self.categ in defs.def_settings['controllers'][con]['categories']
#         ]
#         return items
#
#     def categorylist(self, context):
#         """Create an enum for the controller categories. For phobos preset categories,
#         the phobosIcon is added to the enum.
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#
#         phobosIcon = prev_collections["phobos"]["phobosIcon"].icon_id
#         categories = [t for t in defs.def_subcategories['controllers']]
#
#         icon = ''
#         items = []
#         i = 0
#         for categ in categories:
#             # assign an icon to the phobos preset categories
#             if categ == 'motor':
#                 icon = 'AUTO'
#             else:
#                 #icon = 'GAME'
#                 icon = ''
#
#             items.append((categ, categ, categ, icon, i))
#             i += 1
#
#         return items
#
#     categ : EnumProperty(items=categorylist, description='The controller category')
#
#     controllerType : EnumProperty(items=controllerlist, description='The controller type')
#
#     controllerName : StringProperty(
#         name="Controller name", default='new_controller', description="Name of the controller"
#     )
#
#     addToAll : BoolProperty(
#         name="Add to all",
#         default=True,
#         description="Add a controller to all controllable selected objects",
#     )
#
#     def draw(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         layout = self.layout
#         layout.prop(self, 'controllerName')
#         layout.separator()
#         layout.prop(self, 'categ', text='Sensor category')
#         layout.prop(self, 'controllerType', text='Controller type')
#         layout.prop(self, 'addToAll', icon='PARTICLES')
#
#     def invoke(self, context, event):
#         """
#
#         Args:
#           context:
#           event:
#
#         Returns:
#
#         """
#         return context.window_manager.invoke_props_dialog(self)
#
#     def check(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         return True
#
#     @classmethod
#     def poll(cls, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         return context.active_object and context.active_object.phobostype in defs.controllabletypes
#
#     def execute(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         # match the operator to avoid dangers of eval
#         import re
#
#         opName = addObjectFromYaml(
#             self.controllerName,
#             'controller',
#             self.controllerType,
#             addControllerFromYaml,
#             self.addToAll,
#         )
#         operatorPattern = re.compile('[[a-z][a-zA-Z]*\.]*[a-z][a-zA-Z]*')
#
#         # run the operator and pass on add link (to allow undo both new link and sensor)
#         if operatorPattern.match(opName):
#             eval('bpy.ops.' + opName + "('INVOKE_DEFAULT')")
#         else:
#             log(
#                 'This controller name is not following the naming convention: '
#                 + opName
#                 + '. It can not be converted into an operator.',
#                 'ERROR',
#             )
#         return {'FINISHED'}
#
#
# # [TODO v2.0.0] REVIEW this
# def getControllerParameters(name):
#     """Returns the controller parameters for the controller type with the provided
#     name.
#
#     Args:
#       name(str): the name of the controller type.
#
#     Returns:
#
#     """
#     try:
#         return defs.definitions['controllers'][name]['parameters'].keys()
#     except:
#         return []
#
#
# # [TODO v2.0.0] REVIEW this
# def getDefaultControllerParameters(scene, context):
#     """Returns the default controller parameters for the controller of the active
#     object.
#
#     Args:
#       scene:
#       context:
#
#     Returns:
#
#     """
#     try:
#         name = bpy.context.active_object['motor/controller']
#         return defs.definitions['controllers'][name]['parameters'].values()
#     except:
#         return None
#

# [TODO v2.0.0] REVIEW this
class CreateMimicJointOperator(Operator):
    """Make a number of joints follow a specified joint"""

    bl_idname = "phobos.create_mimic_joint"
    bl_label = "Mimic Joint"
    bl_options = {'REGISTER', 'UNDO'}

    multiplier : FloatProperty(
        name="Multiplier", default=1.0, description="Multiplier for joint mimicry"
    )

    offset : FloatProperty(name="Offset", default=0.0, description="Offset for joint mimicry")

    mimicjoint : BoolProperty(name="Mimic Joint", default=True, description="Create joint mimicry")

    # mimicmotor : BoolProperty(name="Mimic Motor", default=False, description="Create motor mimicry")

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        masterjoint = context.active_object
        objs = filter(
            lambda e: "phobostype" in e and e.phobostype == "link", context.selected_objects
        )

        # apply mimicking for all selected joints
        for obj in objs:
            if obj.name != masterjoint.name:
                if self.mimicjoint:
                    obj["joint/mimic/joint"] = masterjoint.get("joint/name", masterjoint.name)
                    obj["joint/mimic/multiplier"] = self.multiplier
                    obj["joint/mimic/offset"] = self.offset
                # if self.mimicmotor:
                #     obj["motor/mimic/motor"] = nUtils.getObjectName(masterjoint, 'motor')
                #     obj["motor/mimic/multiplier"] = self.multiplier
                #     obj["motor/mimic/offset"] = self.offset
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        ob = context.active_object
        objs = list(
            filter(lambda e: "phobostype" in e and e.phobostype == "link", context.selected_objects)
        )
        return ob is not None and ob.phobostype == 'link' and len(objs) > 1


class AddHeightmapOperator(Operator):
    """Add a heightmap object to the 3D-Cursors location"""

    bl_idname = "phobos.add_heightmap"
    bl_label = "Create heightmap"
    bl_options = {'REGISTER', 'UNDO'}

    name : StringProperty(
        name="Name", description="The name of the new heightmap object", default="heightmap"
    )

    cutNo : IntProperty(
        name="Number of cuts", description="Number of cuts for subdivision", default=100
    )

    strength : FloatProperty(
        name="Displacement strength", description="Strength of the displacement effect", default=0.1
    )

    subsurf : BoolProperty(
        name="Use subsurf", description="Use subsurf modifier to smoothen surface", default=False
    )

    subsurflvl : IntProperty(
        name="Subsurf subdivisions",
        description="Number of divisions for subsurf smoothing",
        default=2,
    )

    filepath : StringProperty(subtype="FILE_PATH")

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout
        layout.prop(self, 'name')
        layout.prop(self, 'cutNo')
        layout.prop(self, 'strength')
        layout.prop(self, 'subsurf')
        if self.subsurf:
            layout.prop(self, 'subsurflvl')

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
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
        if context.view_layer.objects.active:
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
        root = modellinks.deriveLinkfromObject(plane, scale=1.0)

        # set names and custom properties
        # FIXME: what about the namespaces? @HEIGHTMAP (14)
        plane.name = self.name + '_visual::heightmap'
        root.name = self.name + "::heightmap"
        root['entity/type'] = 'heightmap'
        root['entity/name'] = self.name
        root['image'] = os.path.relpath(os.path.basename(self.filepath), bpy.data.filepath)
        root['joint/type'] = 'fixed'

        # select the plane object for further adjustments
        context.view_layer.objects.active = plane

        # FIXME this GUI "hack" does not work, as the buttons context enum is not updated while the operator is running @HEIGHTMAP (30)
        # current_screen = bpy.context.screen.name
        # screen = bpy.data.screens[current_screen]
        # for area in screen.areas:
        #     if area.type == 'PROPERTIES':
        #         area.tag_redraw()
        #         area.spaces[0].context = 'MODIFIER'
        return {'FINISHED'}

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        # create the open file dialog
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


# [TODO v2.0.0] REVIEW this
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
        submodellist = [
            a.name
            for a in bpy.data.groups
            if 'submodeltype' in a and a['submodeltype'] == self.submodeltype
        ]
        return [
            (
                a,
                a.split(':')[1],
                a.split(':')[1].split('/')[0]
                + ' version '
                + a.split(':')[1].split('/')[1]
                + ' submodel',
            )
            for a in submodellist
        ]

    def submodeltypes(self, context):
        """Returns a list of submodel types in the scene for use as enum

        Args:
          context:

        Returns:

        """
        submodellist = [a['submodeltype'] for a in bpy.data.groups if 'submodeltype' in a]
        submodellist = set(submodellist)
        return [(a, a, a + ' submodel type') for a in submodellist]

    submodeltype : EnumProperty(
        name="Submodel type", description="Type of the submodel", items=submodeltypes
    )

    submodelname : EnumProperty(
        name="Submodel name", description="Name of the submodel", items=submodelnames
    )

    instancename : StringProperty(name="Instance name", default='')

    def check(self, context):
        """

        Args:
          context:

        Returns:

        """
        return True

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
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


# [TODO v2.0.0] REVIEW this
class DefineSubmodel(Operator):
    """Define a new submodel from objects"""

    bl_idname = "phobos.define_submodel"
    bl_label = "Define Submodel"
    bl_options = {'REGISTER', 'UNDO'}

    submodelname : StringProperty(
        name="Submodel name", description="Name of the submodel", default='newsubmodel'
    )

    version : StringProperty(
        name="Version name", description="Name of the submodel version", default='1.0'
    )

    submodeltype : EnumProperty(
        items=tuple([(sub,) * 3 for sub in defs.definitions['submodeltypes']]),
        name="Submodel type",
        default="mechanism",
        description="The type for the new submodel",
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
        eUtils.defineSubmodel(self.submodelname, self.submodeltype, self.version)
        return {'FINISHED'}


class AssignSubmechanism(Operator):
    """Assign a submechanism to the model"""

    bl_idname = "phobos.assign_submechanism"
    bl_label = "Assign Submechanism"
    bl_options = {'REGISTER', 'UNDO'}

    # mechanism_type : EnumProperty(
    #    name="Submechanism type",
    #    items=bUtils.compileEnumPropertyList(defs.definitions['submechanisms'].keys()),
    #    )
    # maybe add size in brackets? lambda_mechanism(3) / [3] lambda_mechanism

    # mechanism_category : EnumProperty(
    #    name="Submechanism category",
    #    items=bUtils.compileEnumPropertyList(defs.definitions['submechanisms'].keys()),
    #    )

    linear_chain : BoolProperty(name='Serial Chain', default=False)

    mechanism_name : StringProperty(name='Name')

    contextual_name : StringProperty(name='Contextual name')

    joints = []

    executeMessage = []

    def compileSubmechanismTreeEnum(self, context):
        """

        Args:
          context:

        Returns:

        """
        return bUtils.compileEnumPropertyList(
            defs.definitions['submechanisms'][context.window_manager.mechanismpreview]['joints'][
                'spanningtree'
            ]
        )

    def isLinearChain(self, jointlist):
        """

        Args:
          jointlist:

        Returns:

        """
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

    jointtype0 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype1 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype2 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype3 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype4 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype5 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype6 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype7 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype8 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype9 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype10 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype11 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype12 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype13 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype14 : EnumProperty(items=compileSubmechanismTreeEnum)
    jointtype15 : EnumProperty(items=compileSubmechanismTreeEnum)

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return len(bpy.context.selected_objects) > 0 and any(
            (a.phobostype == 'link' for a in bpy.context.selected_objects)
        )

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        wm = context.window_manager
        layout = self.layout
        nSelectedJoints = len(self.joints)
        layout.label(text='Selection contains {0} joint{1}.'.format(
            nSelectedJoints, "s" if nSelectedJoints is not 1 else ""))
        layout.prop(self, 'linear_chain')
        layout.label(text='(Joints that have not been assigned')
        layout.label(text='to a submechanism will be considered')
        layout.label(text='serial chains automatically)')
        layout.prop(self, 'mechanism_name')
        layout.prop(self, 'contextual_name')
        if not self.linear_chain:
            layout.template_icon_view(wm, 'mechanismpreview', show_labels=True, scale=5.0)
            layout.prop(wm, 'mechanismpreview')
            size = -1 if wm.mechanismpreview == "" else len(
                defs.definitions['submechanisms'][wm.mechanismpreview]['joints']['spanningtree']
            )
            if size == len(self.joints):
                glayout = layout.split()
                c1 = glayout.column(align=True)
                c2 = glayout.column(align=True)
                for i in range(size):
                    c1.label(text=nUtils.getObjectName(self.joints[i], 'joint') + ':')
                    c2.prop(self, "jointtype" + str(i), text='')
            else:
                layout.label(text='Please choose a valid type for selected joints.')
        if len(self.executeMessage) > 0:
            for t in self.executeMessage:
                layout.label(text=t)


    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.executeMessage = []
        self.joints = self.isLinearChain(
            [obj for obj in bpy.context.selected_objects if obj.phobostype == 'link']
        )
        # prepare data used in both cases
        roots = [link for link in self.joints if link.parent not in self.joints]
        if len(roots) != 1:
            self.executeMessage.append("Careful, the selected joints are not all connected")
            if self.linear_chain:
                return {'FINISHED'}
        if self.mechanism_name and self.contextual_name:
            for ob in context.scene.objects:
                if ob.phobostype == "submechanism":
                    if ob.name == self.contextual_name:
                        self.executeMessage.append("Another mechanism with this contextual name already exists")
                        return {'FINISHED'}
            if self.linear_chain:
                base = roots[0]
                parameters = {
                    'type': '{0}R'.format(len(self.joints)),
                    #'jointnames': [j["joint/name"] for j in self.joints], #Is autogenerated
                    'jointnames_spanningtree': [j.get("joint/name",j.name) for j in self.joints],
                    'jointnames_active': [j.get("joint/name",j.name) for j in self.joints],
                    'jointnames_independent': [j.get("joint/name",j.name) for j in self.joints],
                    'name': self.mechanism_name,
                    'contextual_name': self.contextual_name
                }

                subm = hyrodyn.Submechanism(**parameters)
                root = phobos2blender.createSubmechanism(submechanism = subm, linkobj=base)
            elif len(context.window_manager.mechanismpreview) > 0:
                base, freeloader_joints = eUtils.getNearestCommonParent(self.joints)
                if base is None:
                    self.executeMessage.append("The selected links require a common parent link")
                    return {'FINISHED'}
                mechanismdata = defs.definitions['submechanisms'][
                    context.window_manager.mechanismpreview
                ]
                size = len(mechanismdata['joints']['spanningtree'])
                if len(self.joints) == size:
                    jointmap = {
                        getattr(self, 'jointtype' + str(i)): self.joints[i].get("joint/name",self.joints[i].name)
                        for i in range(len(self.joints))
                    }
                    for j in mechanismdata['joints']['spanningtree']:
                        if j not in jointmap:
                            self.executeMessage.append("Define joints")
                            return {'FINISHED'}
                    if len(jointmap) == size: # Every joint is assigned a different type for this submechanism
                        # assign attributes
                        parameters = {
                            'type': mechanismdata['type'],
                            #'subtype': context.window_manager.mechanismpreview,
                            'name': self.mechanism_name,
                            'contextual_name': self.contextual_name,
                            #'jointnames': list(jointmap.values()), # Is autogenerated
                            'jointnames_spanningtree': [
                                jointmap[j] for j in mechanismdata['joints']['spanningtree']
                            ],
                            'jointnames_active': [
                                jointmap[j] for j in mechanismdata['joints']['active']
                            ],
                            'jointnames_independent': [
                                jointmap[j] for j in mechanismdata['joints']['independent']
                            ],
                        }

                        subm = hyrodyn.Submechanism(**parameters)
                        root = phobos2blender.createSubmechanism(submechanism = subm, linkobj=base)
                    else:
                        self.executeMessage.append("Define joints")
                        return {'FINISHED'}
                else:
                    self.executeMessage.append("Got {} joints, {} required".format(len(self.joints), size))
                    return {'FINISHED'}
            else:  # No submechanism selected
                return {'FINISHED'}
        else:
            self.executeMessage.append("Give your submechanism a recognizable name")
            self.executeMessage.append("Contextual name has to be unique")
            return {'FINISHED'}
        self.executeMessage.append("Submechanism assigned")
        return {'FINISHED'}


class SelectSubmechanism(Operator):
    """Select all objects of a submechanism"""

    bl_idname = "phobos.select_submechanism"
    bl_label = "Select Submechanism"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def get_submechanism_roots_static(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return bUtils.compileEnumPropertyList(
            [r['name'] for r in context.scene.objects if r.phobostype == "submechanism"]
        )

    def get_submechanism_roots(self, context):
        """

        Args:
          context:

        Returns:

        """
        return SelectSubmechanism.get_submechanism_roots_static(context)

    submechanism : EnumProperty(
        name="Submechanism",
        description="submechanism which to select",
        items=get_submechanism_roots,
    )

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        # make sure we have a root object with mechanisms
        if not cls.get_submechanism_roots_static(context):
            return False
        return True

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        layout = self.layout

        layout.prop(self, 'submechanism')

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        root = sUtils.getObjectByProperty('contextual_name', self.submechanism)
        jointIDs = root['jointnames_spanningtree']
        jointlist = [
            sUtils.getObjectByProperty('joint/name', id) for id in jointIDs
        ]
        sUtils.selectObjects([root] + jointlist, clear=True, active=0)
        return {'FINISHED'}




# [TODO v2.0.0] REVIEW this
class MergeLinks(Operator):
    """Merge links"""

    bl_idname = "phobos.merge_links"
    bl_label = "Merge Links"
    bl_options = {'REGISTER', 'UNDO'}

    movetotarget : BoolProperty(name='move to target', default=False)

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return (
            context.active_object is not None
            and context.active_object.phobostype == 'link'
            and len(
                [
                    obj
                    for obj in context.selected_objects
                    if obj.phobostype == 'link' and obj != context.active_object
                ]
            )
            > 0
        )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        mergelinks = [
            obj
            for obj in context.selected_objects
            if obj.phobostype == 'link' and obj != context.active_object
        ]
        eUtils.mergeLinks(
            mergelinks, targetlink=context.active_object, movetotarget=self.movetotarget
        )
        return {'FINISHED'}


class SetModelRoot(Operator):
    """Set Model Root"""

    bl_idname = "phobos.set_model_root"
    bl_label = "Set Model Root"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        """

        Args:
          context:

        Returns:

        """
        return context.active_object and context.active_object.phobostype == 'link'

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        newroot = context.active_object
        oldroot = sUtils.getRoot(obj=context.active_object)

        if newroot == oldroot:
            log("Object is already root.", 'INFO')
            return {'CANCELLED'}

        # gather model information from old root
        modelprops = eUtils.getProperties(oldroot, category='model')
        oldroot.pose.bones[0].custom_shape = None

        # assign joint resource to oldroot if applicable
        if 'joint/type' in oldroot:
            oldroot.pose.bones[0].custom_shape = ioUtils.getResource(
                ('joint', oldroot['joint/type'])
            )

        eUtils.restructureKinematicTree(newroot)

        # write model information to new root
        newroot.pose.bones[0].custom_shape = ioUtils.getResource(('link', 'root'))
        eUtils.setProperties(newroot, modelprops, category='model')

        # remove model information from old root
        eUtils.removeProperties(oldroot, ['model/*'])
        return {'FINISHED'}


# TODO remove or use? Give it a dev branch
# class SelectErrorOperator(Operator):
#     """Select an object with check errors"""
#     bl_idname = "phobos.select_error"
#     bl_label = "Select Erroneous Object"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     errorObj : EnumProperty(
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


# [TODO v2.1.0] Make this work again
# class ValidateOperator(Operator):
#     """Check the robot dictionary"""
#
#     bl_idname = "phobos.validate"
#     bl_label = "Validate"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     def execute(self, context):
#         """
#
#         Args:
#           context:
#
#         Returns:
#
#         """
#         messages = {}
#         root = sUtils.getRoot(context.selected_objects[0])
#         model = blender2phobos.deriveRobot(root)
#         # [TODO v2.1.0] Make this work again
#         # vUtils.check_dict(model, defs.definitions['model'], messages)
#         vUtils.checkMessages = messages if len(list(messages.keys())) > 0 else {"NoObject": []}
#         for entry in messages:
#             log("Errors in object " + entry + ":", 'INFO')
#             for error in messages[entry]:
#                 log(error, 'INFO')
#         return {'FINISHED'}


class CalculateMassOperator(Operator):
    """Display mass of the selected objects in a pop-up window"""

    bl_idname = "phobos.calculate_mass"
    bl_label = "Calculate Mass"

    mass : FloatProperty(name='Mass', default=0.0, description="Calculated sum of masses")

    def invoke(self, context, event):
        """

        Args:
          context:
          event:

        Returns:

        """
        inertials = [obj for obj in context.selected_objects if obj.phobostype == 'inertial']
        self.mass = gUtils.calculateSum(inertials, 'mass')
        log("The calculated mass is: " + str(self.mass), 'INFO')
        return context.window_manager.invoke_popup(self)

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        return {'FINISHED'}

    def draw(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.layout.label(text="Sum of masses: " + str(self.mass))


class MeasureDistanceOperator(Operator):
    """Show distance between two selected objects in world coordinates"""

    bl_idname = "phobos.measure_distance"
    bl_label = "Measure Distance"
    bl_options = {'REGISTER'}

    distance : FloatProperty(
        name="Distance",
        default=0.0,
        subtype='DISTANCE',
        unit='LENGTH',
        precision=6,
        description="Distance between objects",
    )

    distVector : FloatVectorProperty(
        name="Distance Vector",
        default=(0.0, 0.0, 0.0),
        subtype='TRANSLATION',
        unit='LENGTH',
        size=3,
        precision=6,
        description="Distance vector between objects",
    )

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        self.distance, self.distVector = gUtils.distance(context.selected_objects)
        log("distance: " + str(self.distance) + ", " + str(self.distVector), "INFO")
        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) == 2

class ParentOperator(Operator):
    """Parent selected objects to active object"""

    bl_idname = "phobos.parent"
    bl_label = "Parent Selection"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """

        Args:
          context:

        Returns:

        """
        parent = context.active_object
        children = context.selected_objects
        for child in children:
            if child != parent:
                eUtils.parentObjectsTo(child, parent)

        return {'FINISHED'}

    @classmethod
    def poll(self, context):
        """

        Args:
          context:

        Returns:

        """
        return len(context.selected_objects) >= 2


classes = (
    DynamicProperty,
    SafelyRemoveObjectsFromSceneOperator,
    MoveToSceneOperator,
    SortObjectsToLayersOperator,
    # [TODO v2.1.0] Re-add AddKinematicChainOperator,
    SetXRayOperator,
    SetPhobosType,
    BatchEditPropertyOperator,
    CreateInterfaceOperator,
    CopyCustomProperties,
    RenameCustomProperty,
    SetGeometryType,
    SmoothenSurfaceOperator,
    EditInertialData,
    GenerateInertialObjectsOperator,
    CreateCollisionObjects,
    SetCollisionGroupOperator,
    DefineJointConstraintsOperator,
    DissolveLink,
    AddMotorOperator,
    CreateLinksOperator,
    AddSensorOperator,
    # [TODO v2.1.0] AddControllerOperator,
    CreateMimicJointOperator,
    AddHeightmapOperator,
    AddSubmodel,
    DefineSubmodel,
    AssignSubmechanism,
    SelectSubmechanism,
    MergeLinks,
    SetModelRoot,
    # [TODO v2.1.0] Make this work again
    # ValidateOperator,
    CalculateMassOperator,
    MeasureDistanceOperator,
    ParentOperator,
)


def register():
    """TODO Missing documentation"""
    print("Registering operators.editing...")
    for classdef in classes:
        bpy.utils.register_class(classdef)


def unregister():
    """TODO Missing documentation"""
    print("Unregistering operators.editing...")
    for classdef in classes:
        bpy.utils.unregister_class(classdef)
