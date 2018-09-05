#!/usr/bin/python
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------------

"""
.. module:: phobos.operators.editing
    :platform: Unix, Windows, Mac
    :synopsis: This module contains operators to manipulate blender objects

.. moduleauthor:: Simon V. Reichel
"""

import bpy
import mathutils
from bpy.types import Operator, PropertyGroup
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

import phobos.defs as defs
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
import phobos.utils.io as ioUtils
from phobos.phoboslog import log


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
        elif isinstance(value, str) and "$selected_objects" in value:
            ptypes = value.split(':')[1:]

            objlist = []
            for ptype in ptypes:
                objlist.extend(
                    [
                        (ptype, obj)
                        for obj in objectlist
                        if (
                            obj.phobostype == ptype
                            or (
                                obj.phobostype == 'link'
                                and 'joint/type' in obj
                                and ptype == 'joint'
                            )
                        )
                    ]
                )
            newanno[key] = objlist
        else:
            newanno[key] = value

    return newanno


class DynamicProperty(PropertyGroup):
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


def addObjectFromYaml(name, phobtype, presetname, execute_func, *args, hideprops=[]):
    """This registers a temporary Operator.
    The data for the properties is provided by the parsed yaml files of the
    specified phobostype. The operator then adds an object and writes the information
    to custom properties.
    The name is changed to fit the blender format of name1_name2_name3.
    """
    # unregister other temporary operators first
    try:
        bpy.utils.unregister_class(TempObjAddOperator)
    except UnboundLocalError:
        pass

    blender_name = name.replace(' ', '_').lower()
    operatorBlenderId = 'phobos.add_{}_{}'.format(phobtype, blender_name)

    # create the temporary operator class
    class TempObjAddOperator(Operator):
        '''Temporary operator to add a generic object.'''

        bl_idname = operatorBlenderId
        bl_label = 'Add {} {}'.format(name, phobtype)
        bl_description = 'Add a {} {}.'.format(name, phobtype)
        bl_options = {'INTERNAL'}

        # this contains all the single entries of the dictionary after invoking
        phobos_data = CollectionProperty(type=DynamicProperty)
        annotation_checks = CollectionProperty(type=DynamicProperty)
        phobostype = phobtype
        obj_name = name
        preset_name = presetname

        def draw(self, context):
            layout = self.layout

            # expose the parameters as the right Property
            for i in range(len(self.phobos_data)):
                name = self.phobos_data[i].name[2:].replace('_', ' ')

                # use the dynamic props name in the GUI, but without the type id
                self.phobos_data[i].draw(layout, name)

            if self.annotation_checks:
                box = layout.box()
                box.label('Include annotations:')
                for i in range(len(self.annotation_checks)):
                    name = self.annotation_checks[i].name[2:].replace('_', ' ')
                    self.annotation_checks[i].draw(box, name)

        def invoke(self, context, event):
            # load the motor definitions of the current motor
            data = defs.definitions[self.phobostype + 's'][self.preset_name]

            # ignore properties which should not show up in the GUI
            hidden_props = ['general'] + hideprops

            # identify the property type for all the stuff in the definition
            unsupported = DynamicProperty.assignDict(
                self.phobos_data.add, data, ignore=hidden_props
            )

            if unsupported:
                for key in unsupported:
                    boolprop = self.annotation_checks.add()
                    boolprop.name = 'b_' + key
                    boolprop.boolProp = False

            # open up the operator GUI
            return context.window_manager.invoke_props_dialog(self)

        def execute(self, context):
            phobos_dict = ioUtils.getDictFromYamlDefs(
                self.phobostype, self.preset_name, self.obj_name
            )
            selected_objs = context.selected_objects

            # store collected object data properties in the props dictionary
            for i in range(len(self.phobos_data)):
                if self.phobos_data[i].name[0] == 'b':
                    store = '$' + str(bool(self.phobos_data[i]['boolProp'])).lower()
                elif self.phobos_data[i].name[0] == 'i':
                    store = self.phobos_data[i]['intProp']
                elif self.phobos_data[i].name[0] == 's':
                    store = self.phobos_data[i]['stringProp']
                elif self.phobos_data[i].name[0] == 'f':
                    store = self.phobos_data[i]['floatProp']

                phobos_dict['props'][self.phobos_data[i].name[2:]] = store

            annotations = {}
            # add annotation objects for other categories
            for custom_anno in self.annotation_checks:
                if custom_anno.boolProp:
                    # parse object dictionaries if "$selected_objects:..." syntax is found
                    annot = defs.definitions[self.phobostype + 's'][self.preset_name][
                        custom_anno.name[2:]
                    ]

                    annotations[custom_anno.name[2:]] = linkObjectLists(annot, selected_objs)

            # let the exectute function handle the object creation
            new_objs, annot_objs, otherobjs = execute_func(
                phobos_dict, annotations, selected_objs, context.active_object, *args
            )

            # select the newly added objects
            sUtils.selectObjects(new_objs + annot_objs + otherobjs, clear=True, active=0)
            bUtils.toggleLayer(defs.layerTypes[self.phobostype], value=True)

            if annot_objs:
                bUtils.toggleLayer(defs.layerTypes['annotation'], value=True)

            # toggle layers for generic objects
            if otherobjs:
                for obj in otherobjs:
                    bUtils.toggleLayer(defs.layerTypes[obj.phobostype], value=True)

            return {'FINISHED'}

    # register the temporary operator and return its blender id
    bpy.utils.register_class(TempObjAddOperator)
    return operatorBlenderId


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
        subcategories = [
            (category,) * 3 for category in sorted(defs.def_subcategories[self.annotationtype])
        ]

        # do not use single categories
        if not subcategories:
            subcategories = [('None',) * 3]
        return subcategories

    def getDeviceTypes(self, context):
        devicetypes = [
            (device,) * 3
            for device in sorted(defs.definitions[self.annotationtype].keys())
            if self.annotationcategories
            in defs.def_settings[self.annotationtype][device]['categories']
        ]

        if not devicetypes:
            devicetypes = [('None',) * 3]
        return devicetypes

    asObject = BoolProperty(
        name="Add as objects", description="Add annotation as object(s)", default=True
    )

    annotationtype = EnumProperty(
        items=getAnnotationTypes, name="Annotation Type", description="Annotation Types"
    )

    annotationcategories = EnumProperty(
        items=getAnnotationCategories,
        name="Categories",
        description="Categories of this annotation type.",
    )

    devicetype = EnumProperty(items=getDeviceTypes, name="Device Type", description="Device Types")

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
            layout.prop(
                self, 'asObject', text='add annotations as object(s)', icon='FORCE_LENNARDJONES'
            )
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
        unsupported = DynamicProperty.assignDict(
            self.annotation_data.add, data, ignore=hidden_props
        )

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
            log(
                "These properties are not supported for generic editing: " + str(list(unsupported)),
                'DEBUG',
            )

    def execute(self, context):
        objects = context.selected_objects
        annotation = defs.definitions[self.annotationtype][self.devicetype]

        # add annotation (objects) to the selected objects
        annot_objects = []
        for obj in objects:
            if self.asObject:
                annot_objects.append(
                    eUtils.addAnnotationObject(
                        obj,
                        annotation,
                        name=obj.name + '_annotation',
                        namespace=self.annotationtype.rstrip('s'),
                    )
                )
            else:
                eUtils.addAnnotation(obj, annotation, namespace=self.annotationtype.rstrip('s'))

        # reselect the original objects and additional annotation objects
        sUtils.selectObjects(objects + annot_objects, clear=True)
        bUtils.toggleLayer(defs.layerTypes['annotation'], value=True)
        return {'FINISHED'}


def register():
    bpy.utils.register_class(DynamicProperty)
    bpy.utils.register_class(AddAnnotationsOperator)


def unregister():
    bpy.utils.unregister_class(DynamicProperty)
    bpy.utils.unregister_class(AddAnnotationsOperator)
