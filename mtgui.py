'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models#

File mtgui.py

Created on 6 Jan 2014

@author: kavonszadkowski
'''

import bpy
from bpy.props import EnumProperty
import marstools.mtdefs


def register():
    bpy.utils.register_class(MARSToolPanel)
    bpy.utils.register_class(MARSObjectPanel)
    bpy.utils.register_class(MARSWorldPanel)

def unregister():
    bpy.utils.unregister_class(MARSToolPanel)
    bpy.utils.unregister_class(MARSObjectPanel)
    bpy.utils.register_class(MARSWorldPanel)



#class MB_GUI(object):
#    '''
#    This class creates a number of GUI elements within Blender that allow to edit
#    a robot model and its MARS properties directly. It provides access to all
#    functionalities of the MARS Blender Tools.
#    '''


def __init__(self, params):
    '''
    Define types and create all GUI elements.
    '''

    bpy.types.Object.MARStype = EnumProperty(
            items = mtdefs.marstypes,
            name = "type",
            description = "MARS object type")



class MARSToolPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_PT_MARS"
    bl_label = "MARS"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'SMOOTH')

    def draw(self, context):
        layout = self.layout

        # Edit Properties Menu
        row_edit = layout.row()
        row_edit.label(text="Edit MARS Properties:")

        split = layout.split()
        col_edit = split.column(align = True)

        col_edit.operator('object.batch_edit_property', text = 'MARS Properties', icon = 'ZOOMIN')
        col_edit.operator('object.batch_edit_property', text = 'Create/Edit Custom Property', icon = 'GREASEPENCIL')

        layout.separator()

        # Validation and Export Menu
        row_export = layout.row()
        row_export.label(text = "Check/Export Robot")

        split = layout.split()
        col_export = split.column(align = True)

        col_export.operator("object.batch_edit_property", text = "Export Robot Model", icon = "PASTEDOWN")



class MARSObjectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_MARS"
    bl_label = "MARS"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw_header(self, context):
        self.layout.label(icon = 'SMOOTH')

    def draw(self, context):

        layout = self.layout

        #the following is used for real pre-defined rather than custom properties
        #row_type.prop(bpy.context.active_object, "type")
        row_type = layout.row()
        row_type.label(icon="OBJECT_DATA")
        #row_type.prop_enum(bpy.context.active_object, '["type"]', "node")
        #row_type.prop_enum(bpy.context.active_object, 'MARStype')
        row_type.prop(bpy.context.active_object, 'MARStype')

        box_props = layout.box()
        for prop in mtdefs.type_properties[bpy.context.active_object.MARStype]:
            #box_props.label(prop)
            if prop in bpy.context.active_object:
                box_props.prop(bpy.context.active_object, '["'+prop+'"]')
            else:
                bpy.context.active_object[prop] = mtdefs.type_properties[bpy.context.active_object.MARStype+"_default"]



class MARSWorldPanel(bpy.types.Panel):
    bl_idname = "WORLD_PT_MARS"
    bl_label = "MARS"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "world"

    def draw_header(self, context):
        self.layout.label(icon = 'SMOOTH')

    def draw(self, context):

        layout = self.layout



# if script is run directly, register contained classes
if __name__ == "__main__":
    register()
