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
Contains the generic Blender operators for adding yaml parsed objects and similar.
"""

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    CollectionProperty,
    StringProperty,
)
from bpy.types import Operator, PropertyGroup
from numpy import isin

from .. import defs as defs
from ..io import phobos2blender, blender2phobos
from ..phoboslog import log
from ..utils import blender as bUtils
from ..utils import editing as eUtils
from ..utils import io as ioUtils
from ..utils import selection as sUtils
from ...io import representation


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
      annotation(dict): annotation dictionary
      objectlist(list(bpy.types.Object): objects to add to the annotation

    Returns:
      : dict -- annotation dictionary with inserted object dictionaries

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


# [TODO v2.1.0] Review this and re-add property adding to AddAnnotationOperator
class DynamicProperty(PropertyGroup):
    """A support class to handle dynamic properties in a temporary operator."""

    name : bpy.props.StringProperty()
    intProp : bpy.props.IntProperty()
    boolProp : bpy.props.BoolProperty()
    stringProp : bpy.props.StringProperty()
    floatProp : bpy.props.FloatProperty()

    STRING = 1
    INT = 2
    BOOL = 3
    FLOAT = 4
    DICT = 5
    valueType : bpy.props.IntProperty()

    isEnabled : bpy.props.BoolProperty()
    isEnabledOption : bpy.props.BoolProperty() # Whether this property can be disabled

    dictName: bpy.props.StringProperty()

    def assignParent(self, name):
        """
        Assign a parent if this property is part of a dict

        Args:
            name: Parent's name

        Returns:

        """
        self.dictName = name

    def getValue(self, properties=[]):
        """
        Args:
            properties: List of all DynamicProperties. Pass this to a dict property.
            Allows it to collect the data

        Returns: This property's value

        """
        if self.valueType == self.INT:
            return self.intProp
        elif self.valueType == self.BOOL:
            return self.boolProp
        elif self.valueType == self.STRING:
            return self.stringProp
        elif self.valueType == self.FLOAT:
            return self.floatProp
        elif self.valueType == self.DICT:
            result = {}
            for prop in properties:
                if prop.dictName == self.name:
                    result[prop.name] = prop.getValue()
            return result

    def isDictElement(self):
        """

        Args:

        Returns: True if this property is part of a dict, False otherwise

        """
        return len(self.dictName) > 0

    def allowDisabling(self):
        """
        Call to allow the user to disable this property

        Args:

        Returns:

        """
        self.isEnabledOption = True

    def assignValue(self, name, value):
        """

        Args:
          name: 
          value: 

        Returns:

        """
        self.isEnabled = True
        self.isEnabledOption = False

        if isinstance(value, bool):
            self.boolProp = value
            self.valueType = self.BOOL
        elif isinstance(value, str):
            self.stringProp = value
            self.valueType = self.STRING
        elif isinstance(value, int):
            self.intProp = value
            self.valueType = self.INT
        elif isinstance(value, float):
            self.floatProp = value
            self.valueType = self.FLOAT
        elif value is None:
            self.intProp = 0
            self.valueType = self.INT
            self.isEnabled = False
            self.allowDisabling()
        elif isinstance(value, dict):
            self.valueType = self.DICT
        else:
            print("DynamicProperty - Unknown type:")
            print(type(value))
            print(value)
        # TODO what about lists?

        self.name = name

    @staticmethod
    def assignDict(addfunc, dictionary, ignore=[]):
        """

        Args:
          addfunc: 
          dictionary: 
          ignore: (Default value = [])

        Returns:

        """
        unsupported = {}
        for propname in dictionary:
            if propname in ignore:
                continue

            subprop = addfunc()
            subprop.assignValue(propname, dictionary[propname])

            # add subcategories
            if isinstance(dictionary[propname], dict):
                unsupported[propname] = dictionary[propname]
                for name, value in dictionary[propname].items():
                    dictprop = addfunc()
                    dictprop.assignValue(name, value)
                    dictprop.assignParent(propname)

        return unsupported

    def draw(self, layout, name):
        """

        Args:
          layout: 
          name: 

        Returns:

        """
        if self.isEnabledOption:
            line = layout.split(factor=0.1)
            line.prop(self, 'isEnabled', text="")
            row = line.row()
            row.enabled = self.isEnabled
        else:
            row = layout

        if len(self.dictName) > 0:
            row.separator(factor=0.03)

        if self.valueType == self.INT:
            row.prop(self, 'intProp', text=name)
        elif self.valueType == self.BOOL:
            row.prop(self, 'boolProp', text=name)
        elif self.valueType == self.STRING:
            row.prop(self, 'stringProp', text=name)
        elif self.valueType == self.FLOAT:
            row.prop(self, 'floatProp', text=name)
        elif self.valueType == self.DICT:
            row.label(text=name+":")

    @staticmethod
    def collectDict(properties):
        """

        Args:
          properties (list of DynamicProperty):

        Returns:
          A dict like the one passed to assignDict with the data the user changed

        """
        result = {}
        for prop in properties:
            if prop.isEnabled and not prop.isDictElement():
                result[prop.name] = prop.getValue(properties)
        return result


def addObjectFromYaml(name, phobtype, presetname, execute_func, *args, hideprops=[]):
    """This registers a temporary Operator.
    The data for the properties is provided by the parsed yaml files of the
    specified phobostype. The operator then adds an object and writes the information
    to custom properties.
    The name is changed to fit the blender format of name1_name2_name3.

    Args:
      name: 
      phobtype: 
      presetname: 
      execute_func: 
      *args: 
      hideprops: (Default value = [])

    Returns:

    """

    blender_name = name.replace(' ', '_').lower()
    operatorBlenderId = 'phobos.add_{}_{}'.format(phobtype, blender_name)

    # create the temporary operator class
    class TempObjAddOperator(Operator):
        """Temporary operator to add a generic object."""

        bl_idname = operatorBlenderId
        bl_label = 'Add {} {}'.format(name, phobtype)
        bl_description = 'Add a {} {}.'.format(name, phobtype)
        bl_options = {'INTERNAL'}

        # this contains all the single entries of the dictionary after invoking
        phobos_data : CollectionProperty(type=DynamicProperty)
        annotation_checks : CollectionProperty(type=DynamicProperty)
        phobostype = phobtype
        obj_name = name
        preset_name = presetname

        def draw(self, context):
            """

            Args:
              context: 

            Returns:

            """
            layout = self.layout

            # expose the parameters as the right Property
            for i in range(len(self.phobos_data)):
                name = self.phobos_data[i].name[2:].replace('_', ' ')

                # use the dynamic props name in the GUI, but without the type id
                self.phobos_data[i].draw(layout, name)

            if self.annotation_checks:
                box = layout.box()
                box.label(text='Include annotations:')
                for i in range(len(self.annotation_checks)):
                    name = self.annotation_checks[i].name[2:].replace('_', ' ')
                    self.annotation_checks[i].draw(box, name)

        def invoke(self, context, event):
            """

            Args:
              context: 
              event: 

            Returns:

            """
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
            """

            Args:
              context: 

            Returns:

            """
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
            bUtils.toggleLayer(self.phobostype, value=True)

            if annot_objs:
                bUtils.toggleLayer('annotation', value=True)

            # toggle layers for generic objects
            if otherobjs:
                for obj in otherobjs:
                    bUtils.toggleLayer(obj.phobostype, value=True)

            return {'FINISHED'}

    # register the temporary operator and return its blender id
    bpy.utils.register_class(TempObjAddOperator)
    return operatorBlenderId


class AddAnnotationsOperator(bpy.types.Operator):
    """Add annotations defined by the Phobos definitions"""

    bl_idname = "phobos.add_annotations"
    bl_label = "Add Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    category : StringProperty(
        name="Annotation Type", description="Annotation Types"
    )

    include_parent : BoolProperty(
        name="Include parent link", default=False,
        description="By using the string key $parent you can include the name of the parent link in your annotations.\n"
                    "Using $parent.{property} let's you use the parents phobos.core properties e.g. the joint name"
    )

    include_transform : BoolProperty(
        name="Includes transformation", default=False,
        description="By using the string key $transform you can include the name of the parent link in your annotations.\n"
                    "Using &transform.matrix/position/rotation_euler/quaternion let's you choose in which way it is stored."
    )

    multiple_entries : BoolProperty(
        name="Multiple Entries", default=True,
        description="If you have multiple annotations in this category"
    )

    name : StringProperty(
        name="Annotation name", description="This name will be used for the annotation, if there are multiple annotations in this category."
    )

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        """

        Args:
          context: 

        Returns:

        """
        layout = self.layout
        layout.label(text="The category for your annotation")
        layout.prop(self, 'category')
        layout.prop(self, 'multiple')
        layout.prop(self, 'name')

        layout.prop(self, 'include_parent')
        layout.prop(self, 'include_transform')

        layout.label(text="After you have created the annotation object, you can:\n"
                          "- Position it in the 3d view\n"
                          "- Parent it to other objects (as Bone Relative)\n"
                          "- Define its properties in the custom property panel\n"
                          "  To create nested entries use the prop/nest/key syntax for the property name")

    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """

        parent = None
        if not hasattr(context.active_object, "phobostype"):
            log("Annotation will not be parented to the active object, as it is no phobos object", "WARN")
        elif self.include_parent:
            parent = context.active_object
        phobos2blender.createAnnotation(
            representation.GenericAnnotation(
                GA_category=self.category,
                GA_name=self.name,
                GA_parent=parent if parent else None,
                GA_parent_type=parent.phobostype if parent else None,
                GA_transform=blender2phobos.deriveObjectPose(context.active_object)
                if context.active_object is not None and self.include_transform else None
            )
        )
        bUtils.toggleLayer('annotation', value=True)
        return {'FINISHED'}


def register():
    """TODO Missing documentation"""
    bpy.utils.register_class(AddAnnotationsOperator)


def unregister():
    """TODO Missing documentation"""
    bpy.utils.unregister_class(AddAnnotationsOperator)
