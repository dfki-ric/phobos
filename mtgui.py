'''
MARS Blender Tools - a Blender Add-On to work with MARS robot models

File mtgui.py

Created on 6 Jan 2014

@author: Kai von Szadkowski

Copy this add-on to your Blender add-on folder and activate it
in your preferences to gain instant (virtual) world domination.
You may use the provided install shell script.
'''

import bpy
from bpy.types import Operator
from bpy.props import EnumProperty, BoolProperty, StringProperty
import marstools.mtdefs as mtdefs
import marstools.mtsensors as mtsensors
import marstools.mtcontrollers as mtcontrollers
import marstools.mtutility as mtutility


def register():
    print("Registering mtgui...")
    bpy.types.Object.MARStype = EnumProperty(
            items = mtdefs.marstypes,
            name = "type",
            description = "MARS object type")
    print("    Added 'MARStype' to Object properties.")

    bpy.types.World.showBodies = BoolProperty(name = "showBodies")
    bpy.types.World.showJoints = BoolProperty(name = "showJoints")
    bpy.types.World.showConstraints = BoolProperty(name = "showConstraints")
    bpy.types.World.showJointSpheres = BoolProperty(name = "showJointSpheres")
    bpy.types.World.showSensors = BoolProperty(name = "showSensors")
    bpy.types.World.showNames = BoolProperty(name = "showNames")
    bpy.types.World.showDecorations = BoolProperty(name = "showDecorations")
    #setWorldView([True, True, True, True, True, False, False])

    # These may be optional
    #bpy.utils.register_class(MARSToolPanel)
    #bpy.utils.register_class(MARSObjectPanel)
    #bpy.utils.register_class(MARSWorldPanel)

def unregister():
    print("Unregistering mtgui...")
    #bpy.utils.unregister_class(MARSToolPanel)
    #bpy.utils.unregister_class(MARSObjectPanel)
    #bpy.utils.register_class(MARSWorldPanel)


class MessageOperator(bpy.types.Operator):
    bl_idname = "error.message"
    bl_label = "Displays a message in a window"
    type = StringProperty()
    message = StringProperty()

    def execute(self, context):
        self.report({'INFO'}, self.message)
        print(self.message)
        return {'FINISHED'}

    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_popup(self, width=400, height=200)

    def draw(self, context):
        self.layout.label("MARS Tools")
        row = self.layout#.split(0.25)
        row.prop(self, "type")
        row.prop(self, "message")
        #row = self.layout#.split(0.80)
        row.label("")
        row.operator("error.ok")

class OkOperator(bpy.types.Operator):
    bl_idname = "error.ok"
    bl_label = "OK"
    def execute(self, context):
        return {'FINISHED'}



class setLayersOperator(Operator):#
    """setLayersOperator"""
    bl_idname = "world.set_layers"
    bl_label = "Set active Layers according to MARS world data."

    def execute(self, context):
        layers = [False]*20
        layers[0] = bpy.data.worlds[0].showBodies
        layers[1] = bpy.data.worlds[0].showJoints
        layers[2] = bpy.data.worlds[0].showJointSpheres
        layers[3] = bpy.data.worlds[0].showSensors
        layers[4] = bpy.data.worlds[0].showDecorations
        layers[5] = bpy.data.worlds[0].showConstraints
        layers[6] = bpy.data.worlds[0].showNames
        onetrue = False
        for b in layers:
            onetrue = onetrue or b
        print (onetrue)
        if not onetrue:
            bpy.data.worlds[0].showBodies = True
            layers[0] = True
        bpy.context.scene.layers = layers
        return {'FINISHED'}



def setWorldView(b):
    bpy.data.worlds[0].showBodies = b[0]
    bpy.data.worlds[0].showJoints = b[1]
    bpy.data.worlds[0].showJointSpheres = b[2]
    bpy.data.worlds[0].showSensors = b[3]
    bpy.data.worlds[0].showDecorations = b[4]
    bpy.data.worlds[0].showConstraints = b[5]
    bpy.data.worlds[0].showNames = b[6]
    applyWorldView()

def applyWorldView():
    layers = [False]*20
    layers[0] = bpy.data.worlds[0].showBodies
    layers[1] = bpy.data.worlds[0].showJoints
    layers[2] = bpy.data.worlds[0].showJointSpheres
    layers[3] = bpy.data.worlds[0].showSensors
    layers[4] = bpy.data.worlds[0].showDecorations
    layers[5] = bpy.data.worlds[0].showConstraints
    layers[6] = bpy.data.worlds[0].showNames
    bpy.context.scene.layers = layers


class MARSToolPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_PT_MARS"
    bl_label = "MARStools panel for general functionality"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'SMOOTH')

    def draw(self, context):
        layout = self.layout

        # Edit Properties Menu
        layout.label(text="Edit MARS Properties:")
        col_edit = layout.column(align = True)

        col_edit.operator('object.mt_update_models', text = 'Update MARS model', icon = 'FILE_REFRESH')
        col_edit.operator('object.mt_batch_property', text = 'Edit Custom Property', icon = 'GREASEPENCIL')

        layout.separator()

        # Inspection Menu
        layout.label(text = "Inspect Robot", icon = 'VIEWZOOM')
        inlayout = layout.split()
        linspect1 = inlayout.column(align = True)
        linspect1.operator('object.mt_check_model', text = 'Check model validity')
        linspect1.operator('object.mt_calculate_mass', text = 'Show Mass')
        linspect1.operator('object.mt_select_root', text = 'Select Root')
        linspect1.operator('object.mt_select_model', text = 'Select Robot')
        linspect1.operator('object.mt_name_model', text = 'Name Robot')
        linspect2 = inlayout.column(align = True)
        linspect2.operator('object.mt_show_motor_types', text = "Show Motor Types")
        linspect2.operator('object.mt_unshow_motor_types', text = "Unshow Motor Types")


class MARSToolModelPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_MODEL_PT_MARS"
    bl_label = "MARStools panel for Bodies & Joints"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'MOD_ARMATURE')

    def draw(self, context):
        layout = self.layout.split()
        c1 = layout.column(align = True)
        c1.operator('object.add_joints', text = "Add joint(s)")
        c2 = layout.column(align = True)
        c2.operator('object.derive_joint_spheres', text = "Derive Joint Spheres")


class MARSToolSenConPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_SENCON_PT_MARS"
    bl_label = "MARStools panel for Sensors & Controllers"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'GAME')

    def draw(self, context):
        row_sensors = self.layout
        # create sensor creation buttons
        row_sensors.label(text="Add Sensors / Controllers")
        sensor_split = row_sensors.split()

        n_sensortypes = int(len(mtdefs.sensorTypes))
        half_n_sensortypes = int(n_sensortypes/2)
        col_sensor_1 = sensor_split.column(align=True)
        for i in range(half_n_sensortypes):#sensor in mtdefs.sensorTypes:
            sensor = mtdefs.sensorTypes[i]
            #col_sensor_1.operator('object.mt_add_sensor_'+sensor, text=sensor)
            col_sensor_1.operator('object.mt_add_sensor', text=sensor).sensor_type = sensor
        col_sensor_2 = sensor_split.column(align=True)
        for i in range(n_sensortypes-half_n_sensortypes):
            sensor = mtdefs.sensorTypes[i+half_n_sensortypes]
            col_sensor_2.operator('object.mt_add_sensor', text=sensor).sensor_type = sensor
            #col_sensor_2.operator('object.mt_add_sensor_'+sensor, text=sensor)
        row_sensors.operator("object.mt_add_controller", text="Controller")


class MARSToolVisPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_VIS_PT_MARS"
    bl_label = "MARStools panel for Visibility"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'VISIBLE_IPO_ON')

    def draw(self, context):
        lvis = self.layout
        # Visibility
        lsplit = lvis.column(align=True)
        #lsplit = layout.split()
        lsplit.prop(bpy.data.worlds[0], "showBodies")
        lsplit.prop(bpy.data.worlds[0], "showJoints")
        lsplit.prop(bpy.data.worlds[0], "showJointSpheres")
        lsplit.prop(bpy.data.worlds[0], "showSensors")
        lsplit.prop(bpy.data.worlds[0], "showDecorations")
        lsplit.prop(bpy.data.worlds[0], "showConstraints")
        lsplit.prop(bpy.data.worlds[0], "showNames")
        lsplit.operator('world.set_layers', text='Apply Visibility')


class MARSToolExportPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for MARS options"""
    bl_idname = "TOOLS_ZEXPORT_PT_MARS"
    bl_label = "MARStools panel for Exporting"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    def draw_header(self, context):
        self.layout.label(icon = 'SMOOTH')

    def draw(self, context):
        group_export = self.layout
        #export robot model options
        group_export.prop(bpy.data.worlds[0], "path")
        group_export.prop(bpy.data.worlds[0], "filename")
        group_export.prop(bpy.data.worlds[0], "exportBobj")
        group_export.prop(bpy.data.worlds[0], "exportMesh")
        group_export.operator("object.mt_export_robot", text = "Export Robot Model", icon = "PASTEDOWN")


class MARSObjectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_MARS"
    bl_label = "MARStools Object panel displaying custom properties"
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
            #else:
            #    bpy.context.active_object[prop] = mtdefs.type_properties[bpy.context.active_object.MARStype+"_default"]



# class MARSWorldPanel(bpy.types.Panel):
#     bl_idname = "WORLD_PT_MARS"
#     bl_label = "MARS"
#     bl_space_type = 'PROPERTIES'
#     bl_region_type = 'WINDOW'
#     bl_context = "world"
#
#     def draw_header(self, context):
#         self.layout.label(icon = 'SMOOTH')
#
#     def draw(self, context):
#
#         layout = self.layout
#
# #         layout.label(text="Export the Model:")
# #         group_export = layout.box()
# #         group_export.prop(bpy.data.worlds[0], "path")
# #         group_export.prop(bpy.data.worlds[0], "filename")
# #         group_export.prop(bpy.data.worlds[0], "exportBobj")
# #         group_export.prop(bpy.data.worlds[0], "exportMesh")
# #         group_export.operator("object.mt_export_robot", text = "Export Robot Model", icon = "PASTEDOWN")



# if script is run directly, register contained classes
if __name__ == "__main__":
    register()
