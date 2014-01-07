'''
Created on 6 Jan 2014

@author: kavonszadkowski
'''

import bpy
from bpy.types import Operator
from bpy.props import StringProperty


def register():
    bpy.utils.register_class(BatchEditPropertyOperator)
    #bpy.types.VIEW3D_MT_object.append(add_object_button)


def unregister():
    bpy.utils.unregister_class(BatchEditPropertyOperator)
    #del bpy.types.VIEW3D_MT_object.append(add_object_button)



class BatchEditPropertyOperator(Operator):
    """Batch-Edit Property Operator"""
    bl_idname = "object.batch_edit_property"
    bl_label = "Edit custom property"
    bl_options = {'REGISTER', 'UNDO'}

    property_name = StringProperty(
        name = "property_name",
        default = "",
        description = "custom property name")

    property_value = StringProperty(
        name = "property_value",
        default = "",
        description = "custom property value")

    def execute(self, context):
        for obj in bpy.context.selected_objects:
            obj[self.property_name] = self.property_value
        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.mode == 'OBJECT'


# the following code is used to directly add buttons to current operator menu
# - we don't need that if we create a custom toolbar with pre-defined buttons
# def add_object_button(self, context):
#     self.layout.operator(
#         BatchEditPropertyOperator.bl_idname,
#         text=BatchEditPropertyOperator.__doc__,
#         icon='PLUGIN')



# if script is run directly, register contained classes
if __name__ == "__main__":
    register()