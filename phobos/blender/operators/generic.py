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
    FloatProperty
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
from ..phobosgui import dynamicLabel

annotationEditOnly = False

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

    def getValue(self, properties=[], boolAsString=False):
        """
        Args:
            properties: List of all DynamicProperties. Pass this to a dict property.
            Allows it to collect the data
            boolAsString:

        Returns: This property's value

        """
        if self.valueType == self.INT:
            return self.intProp
        elif self.valueType == self.BOOL:
            if boolAsString:
                return "True" if self.boolProp else "False"
            else:
                return self.boolProp
        elif self.valueType == self.STRING:
            return self.stringProp
        elif self.valueType == self.FLOAT:
            return self.floatProp
        elif self.valueType == self.DICT:
            result = {}
            for prop in properties:
                if prop.dictName == self.name:
                    result[prop.name] = prop.getValue(boolAsString=boolAsString)
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

    def assignValue(self, name, value, boolAsString=False):
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
            if boolAsString and value in ["True", "False"]:
                self.boolProp = True if value == "True" else False
                self.valueType = self.BOOL
            else:
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
    def assignDict(addfunc, dictionary, ignore=[], boolAsString=False):
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
            subprop.assignValue(propname, dictionary[propname], boolAsString=boolAsString)

            # add subcategories
            if isinstance(dictionary[propname], dict):
                unsupported[propname] = dictionary[propname]
                for name, value in dictionary[propname].items():
                    dictprop = addfunc()
                    dictprop.assignValue(name, value, boolAsString=boolAsString)
                    dictprop.assignParent(propname)

        return unsupported

    @classmethod
    def drawAll(cls, properties, layout):
        """

        Args:
            properties: List of DynamicProperties
            layout:

        Returns:

        """

        for i in range(len(properties)):
            prop = properties[i]
            if not prop.isDictElement():
                prop.draw(layout, properties)

    def draw(self, layout, properties=[]):
        """

        Args:
          layout: UILayout to draw to
          properties: List of DynamicProperties, to draw children of this property

        Returns:

        """
        if self.isEnabledOption:
            line = layout.split(factor=0.1)
            line.prop(self, 'isEnabled', text="")
            row = line.row()
            row.enabled = self.isEnabled
        else:
            row = layout

        # if len(self.dictName) > 0:
        #     row = row.split(factor=0.03)
        #     row.separator()

        if self.valueType == self.INT:
            row.prop(self, 'intProp', text=self.name)
        elif self.valueType == self.BOOL:
            row.prop(self, 'boolProp', text=self.name)
        elif self.valueType == self.STRING:
            row.prop(self, 'stringProp', text=self.name)
        elif self.valueType == self.FLOAT:
            row.prop(self, 'floatProp', text=self.name)
        elif self.valueType == self.DICT:
            row = row.box()
            row.label(text=self.name+":")
            numElem = 0
            for prop in properties:
                if prop.dictName == self.name:
                    prop.draw(row, properties)
                    numElem += 1
            row.label(text=f"{numElem} elements")

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


# TODO: Delete?
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


class AnnotationsOperator(bpy.types.Operator):
    """Add annotations defined by the Phobos definitions"""

    bl_idname = "phobos.annotations"
    bl_label = "Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    ADD_PROPERTY_TEXT = "Select to add"

    ANNOTATION_ROOT = "Annotation root"

    PARAMS = ["$include_parent", "$include_transform", "GA_category", "GA_name", "phobosmatrixinfo",
              "phobostype"]

    TYPES = [(ADD_PROPERTY_TEXT, -1), ("Text", DynamicProperty.STRING),
             ("Number", DynamicProperty.FLOAT), ("Boolean", DynamicProperty.BOOL)]

    TYPES_ROOT = TYPES + [("Dictionary", DynamicProperty.DICT)]

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

    visual_size : FloatProperty(
        name="Visual size", default=1.0
    )

    objectReady = False

    def propertyTypes(self, context):
        items = []
        for name, id in (AnnotationsOperator.TYPES_ROOT if self.add_property_root ==
                         AnnotationsOperator.ANNOTATION_ROOT else AnnotationsOperator.TYPES):
            items.append((name, name, name))
        return items

    def propertyRoots(self, context):
        roots = [(AnnotationsOperator.ANNOTATION_ROOT, AnnotationsOperator.ANNOTATION_ROOT,
                  AnnotationsOperator.ANNOTATION_ROOT)]
        for i in range(len(self.custom_properties)):
            prop = self.custom_properties[i]
            if prop.valueType == DynamicProperty.DICT:
                roots.append((prop.name, prop.name, "Dictionary "+prop.name))
        return roots


    add_property_root : EnumProperty(name="Property root", items=propertyRoots,
                                     description='The new property will be added to this dictionary, '
                                                 'list or the annotation root')

    add_property_name : StringProperty(name="Name", description="Property name")

    add_property : EnumProperty(name="Type", items=propertyTypes, description='Add property')

    custom_properties: CollectionProperty(type=DynamicProperty)

    modify : BoolProperty(
        name="modify",
        default=False
    )

    def getPropertyTypeID(self, name):
        for n, id in AnnotationsOperator.TYPES_ROOT:
            if name == n:
                return id

    def getPropertyByName(self, name, root):
        if root == self.ANNOTATION_ROOT:
            root = ""
        for i in range(len(self.custom_properties)):
            prop = self.custom_properties[i]
            n = prop.name
            r = prop.dictName
            if n == name and r == root:
                return self.custom_properties[i]

    def invoke(self, context, event):
        """

        Args:
          context: 
          event: 

        Returns:

        """
        global annotationEditOnly
        self.modify = annotationEditOnly
        if self.modify:
            ob = context.active_object

            # Read parameters

            self.name = ob["GA_name"]
            self.category = ob["GA_category"]
            self.include_parent = ob["$include_parent"]
            self.include_transform = ob["$include_transform"]

            # Read visual size
            scale = ob.scale
            if len(set(scale)): # All values are equal
                self.visual_size = scale[0]
            else:
                self.visual_size = 0

            # Read custom properties
            data = {}
            for key, value in ob.items():
                if key not in self.PARAMS:
                    if isinstance(value, (int, float)):
                        value = float(value)
                    elif isinstance(value, str):
                        pass
                    elif isinstance(value, list):
                        pass
                    else:
                        value = value.to_dict()

                    data[key] = value
            DynamicProperty.assignDict(self.custom_properties.add, data, boolAsString=True)
            self.objectReady = True
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        """

        Args:
          context: 

        Returns:

        """
        layout = self.layout
        layout.label(text="Type (category) and name of your annotation")
        layout.prop(self, 'category')
        #layout.prop(self, 'multiple_entries')
        layout.prop(self, 'name')

        layout.prop(self, 'visual_size')

        layout.prop(self, 'include_parent')
        layout.prop(self, 'include_transform')

        if not self.objectReady:

            dynamicLabel(text="After you have created the annotation object, you can: \n"
                              "- Position it in the 3d view \n"
                              "- Parent it to other objects \n"
                              "- Define its properties in the custom property panel\n"
                              "  To create nested entries use the prop/nest/key syntax for the property name",
                         uiLayout=layout, width=500)

        layout.separator()
        layout.label(text="Custom properties")

        #if len(self.propertyRoots(context)) > 1:
        layout.prop(self, 'add_property_root')

        c = layout.split()
        c1, c2 = c.column(), c.column()

        if self.add_property != self.ADD_PROPERTY_TEXT:
            ID = self.getPropertyTypeID(self.add_property)
            newName = self.add_property_name
            # Check if a property with this name already exists
            if newName and self.getPropertyByName(newName, self.add_property_root) is None:
                new_prop = self.custom_properties.add()
                new_prop.valueType = ID
                new_prop.name = newName
                # Assign dict
                if self.add_property_root != self.ANNOTATION_ROOT:
                    new_prop.assignParent(self.add_property_root)

            else:
                c1.alert = True
            self.add_property = self.ADD_PROPERTY_TEXT

        c1.prop(self, 'add_property_name')
        c2.prop(self, 'add_property')

        DynamicProperty.drawAll(self.custom_properties, layout)




    def execute(self, context):
        """

        Args:
          context: 

        Returns:

        """

        if not self.modify:
            parent = None
            if not hasattr(context.active_object, "phobostype"):
                log("Annotation will not be parented to the active object, as it is no phobos object", "WARN")
            elif self.include_parent:
                parent = context.active_object
            ob = phobos2blender.createAnnotation(
                representation.GenericAnnotation(
                    GA_category=self.category,
                    GA_name=self.name,
                    GA_parent=parent if parent else None,
                    GA_parent_type=parent.phobostype if parent else None,
                    GA_transform=blender2phobos.deriveObjectPose(context.active_object)
                    if context.active_object is not None and self.include_transform else None
                ),
                size=self.visual_size
            )
            bUtils.toggleLayer('annotation', value=True)

        else:
            ob = context.active_object
            # Rescale
            if self.visual_size > 0:
                ob.scale = (self.visual_size, ) * 3

            # Update parameters
            ob["GA_category"] = self.category
            ob["GA_name"] = self.name
            ob["$include_parent"] = self.include_parent
            ob["$include_transform"] = self.include_transform

        # Write custom properties to object
        for i in range(len(self.custom_properties)):
            prop = self.custom_properties[i]
            if not prop.isDictElement():
                name = prop.name
                value = prop.getValue(self.custom_properties, boolAsString=True)
                ob[name] = value

        self.objectReady = True
        return {'FINISHED'}

class EditAnnotationsOperator(bpy.types.Operator):
    """Modify annotations"""

    bl_idname = "phobos.edit_annotations"
    bl_label = "Edit Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        global annotationEditOnly
        annotationEditOnly = True
        bpy.ops.phobos.annotations('INVOKE_DEFAULT')
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.phobostype == "annotation"


class AddAnnotationsOperator(bpy.types.Operator):
    """Modify annotations"""

    bl_idname = "phobos.add_annotations"
    bl_label = "Add Annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        global annotationEditOnly
        annotationEditOnly = False
        bpy.ops.phobos.annotations('INVOKE_DEFAULT')
        return {'FINISHED'}


def register():
    """TODO Missing documentation"""


def unregister():
    """TODO Missing documentation"""
