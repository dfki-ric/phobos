#!/usr/bin/python

"""
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

File phobosgui.py

Created on 6 Jan 2014

@author: Kai von Szadkowski
"""

import bpy
from bpy.props import IntProperty, FloatProperty, BoolProperty, EnumProperty, StringProperty
from . import defs


def register():
    print("Registering gui...")
    bpy.types.Object.phobostype = EnumProperty(
        items=defs.phobostypes,
        name="type",
        description="Phobos object type")
    print("    Added 'phobostype' to Object properties.")
    # bpy.types.Object.lastchanged = StringProperty(
    #        default = '',
    #        name = "lastchanged",
    #        description = "Iso format datetime string of last change event")

    bpy.types.World.showBodies = BoolProperty(name="showBodies", update=SetVisibleLayers)
    bpy.types.World.showJoints = BoolProperty(name="showJoints", update=SetVisibleLayers)
    bpy.types.World.showConstraints = BoolProperty(name="showConstraints", update=SetVisibleLayers)
    bpy.types.World.showJointSpheres = BoolProperty(name="showJointSpheres", update=SetVisibleLayers)
    bpy.types.World.showSensors = BoolProperty(name="showSensors", update=SetVisibleLayers)
    bpy.types.World.showNames = BoolProperty(name="showNames", update=SetVisibleLayers)
    bpy.types.World.showDecorations = BoolProperty(name="showDecorations", update=SetVisibleLayers)
    #bpy.types.World.showMotorTypes = BoolProperty(name = "showMotorTypes", update=showMotorTypes)
    bpy.types.World.manageLayers = BoolProperty(name="manage layers", update=manageLayers)
    bpy.types.World.useDefaultLayers = BoolProperty(name="use default layers", update=useDefaultLayers)
    bpy.types.World.linkLayer = IntProperty(name="link", update=manageLayers)

    bpy.types.World.path = StringProperty(name='path', default='.', update=updateExportPath)
    bpy.types.World.decimalPlaces = IntProperty(name="decimalPlaces",
                                                description="Number of decimal places to export",
                                                default=6)
    bpy.types.World.relativePath = BoolProperty(name='relative path', default=True)
    bpy.types.World.heightmapMesh = BoolProperty(name='export heightmap as mesh', default=False)
    bpy.types.World.useBobj = BoolProperty(name="useBobj", update=updateExportOptions)
    bpy.types.World.useObj = BoolProperty(name="useObj", update=updateExportOptions)
    bpy.types.World.useStl = BoolProperty(name="useStl", update=updateExportOptions)
    bpy.types.World.useDae = BoolProperty(name="useDae", update=updateExportOptions)
    bpy.types.World.exportMeshes = BoolProperty(name="exportMeshes", update=updateExportOptions)
    bpy.types.World.exportTextures = BoolProperty(name="exportTextures", update=updateExportOptions)
    bpy.types.World.exportCustomData = BoolProperty(name="exportCustomData", update=updateExportOptions)
    bpy.types.World.exportMARSscene = BoolProperty(name="exportMARSscene", update=updateExportOptions)
    bpy.types.World.exportSMURF = BoolProperty(name="exportSMURF", default=True, update=updateExportOptions)
    bpy.types.World.exportURDF = BoolProperty(name="exportURDF", default=True, update=updateExportOptions)
    bpy.types.World.exportSRDF = BoolProperty(name="exportSRDF", default=True)
    bpy.types.World.exportYAML = BoolProperty(name="exportYAML", update=updateExportOptions)
    bpy.types.World.structureExport = BoolProperty(name="structureExport", default=False, description="Create structured subfolders")
    bpy.types.World.sceneName = StringProperty(name="sceneName")

    #bpy.types.World.gravity = FloatVectorProperty(name = "gravity")


def unregister():
    print("Unregistering gui...")


class MessageOperator(bpy.types.Operator):
    bl_idname = "error.message"
    bl_label = "Display a message in a window"
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
        self.layout.label("Phobos")
        row = self.layout  # .split(0.25)
        row.prop(self, "type")
        row.prop(self, "message")
        # row = self.layout#.split(0.80)
        row.label("")
        row.operator("error.ok")


class OkOperator(bpy.types.Operator):
    bl_idname = "error.ok"
    bl_label = "OK"

    def execute(self, context):
        return {'FINISHED'}


def updateExportOptions(self, context):
    if bpy.data.worlds[0].exportSMURF and not bpy.data.worlds[0].exportURDF:
        bpy.data.worlds[0].exportURDF = True


def updateExportPath(self, context):
    if not bpy.data.worlds[0].path.endswith('/'):
        bpy.data.worlds[0].path += '/'


def SetVisibleLayers(self, context):
    """Set active Layers according to Phobos world data."""
    layers = [False] * 20
    layers[0] = bpy.data.worlds[0].showBodies
    layers[1] = bpy.data.worlds[0].showJoints
    layers[2] = bpy.data.worlds[0].showJointSpheres
    layers[3] = bpy.data.worlds[0].showSensors
    layers[4] = bpy.data.worlds[0].showDecorations
    layers[5] = bpy.data.worlds[0].showConstraints
    onetrue = False
    for b in layers:
        onetrue = onetrue or b
    if not onetrue:
        bpy.data.worlds[0].showBodies = True
        layers[0] = True
    bpy.context.scene.layers = layers
    for obj in bpy.data.objects:
        obj.show_name = bpy.data.worlds[0].showNames


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
    layers = [False] * 20
    layers[0] = bpy.data.worlds[0].showBodies
    layers[1] = bpy.data.worlds[0].showJoints
    layers[2] = bpy.data.worlds[0].showJointSpheres
    layers[3] = bpy.data.worlds[0].showSensors
    layers[4] = bpy.data.worlds[0].showDecorations
    layers[5] = bpy.data.worlds[0].showConstraints
    layers[6] = bpy.data.worlds[0].showNames
    bpy.context.scene.layers = layers


def manageLayers(self, context):
    if bpy.data.worlds[0].manageLayers:
        pass  # TODO: not so important


def useDefaultLayers(self, context):
    pass  # TODO: not so important


# def showMotorTypes(self, context):
# """Changes materials of joints to indicate different motor types."""
#     if bpy.data.worlds[0].showMotorTypes:
#         types = {}
#         n_indicators = 0
#         for obj in bpy.context.selected_objects:
#             if obj.phobostype == "joint":
#                 if "spec_motor" in obj:
#                     if not (obj["spec_motor"] in types):
#                         n_indicators += 1
#                         types[obj["spec_motor"]] = "indicator" + str(n_indicators)
#                     if not types[obj["spec_motor"]] in obj.data.materials:
#                         obj.data.materials.append(bpy.data.materials[types[obj["spec_motor"]]])
#                         obj.data.materials.pop(0, update_data=True)
#         bpy.data.scenes[0].update()
#     else:
#         for obj in bpy.context.selected_objects:
#             if obj.phobostype == "joint":
#                 obj.data.materials.append(bpy.data.materials["Joint Discs"])
#                 obj.data.materials.pop(0, update_data=True)
#     bpy.data.scenes[0].update()


class PhobosPanel(bpy.types.Panel):
    """A Custom Panel in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_PT_PHOBOS"
    bl_label = "phobos: Model Editing"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'
    #bl_context = ''

    def draw_header(self, context):
        self.layout.label(icon='SMOOTH')

    def draw(self, context):
        layout = self.layout

        # Robot Model Menu
        layout.label(text="Robot Model:")
        inlayout = layout.split()
        rc1 = inlayout.column(align=True)
        rc1.operator('object.phobos_add_chain', text='Define Kinematic Chain', icon='CONSTRAINT')
        rc2 = inlayout.column(align=True)
        rc2.operator('object.phobos_name_model', text='Name Robot')
        rc2.operator('object.phobos_toggle_namespaces', text='Toggle Namespaces')

        #Scene Menu
        layout.separator()
        layout.label(text="Scene Editing", icon="WORLD")
        iinlayout = layout.split()
        ic1 = iinlayout.column(align=True)
        ic1.operator("object.phobos_add_heightmap", text="Create Heightmap")
        ic2 = iinlayout.column(align=True)

        # Inspection Menu
        layout.separator()
        layout.label(text="Inspect Robot", icon='VIEWZOOM')
        iinlayout = layout.split()
        ic1 = iinlayout.column(align=True)
        ic1.operator('object.phobos_show_distance', text='Measure Distance')
        ic1.operator('object.phobos_check_dict', text='Check Robot Dictionary')
        ic2 = iinlayout.column(align=True)
        ic2.operator('object.phobos_set_xray', text='X-Ray View')
        ic2.operator('object.phobos_select_error', text="Select Erroneous Object")

        # Selection Menu
        layout.separator()
        layout.label(text="Selection(s)", icon='HAND')
        sinlayout = layout.split()
        sc1 = sinlayout.column(align=True)
        sc1.operator('object.phobos_select_root', text='Select Root')
        sc1.operator('object.phobos_select_model', text='Select Robot')
        sc2 = sinlayout.column(align=True)
        sc2.operator('object.phobos_select_objects_by_phobostype', text="Select by Phobostype")
        sc2.operator('object.phobos_select_objects_by_name', text="Select by Name")

        # Pose Menu
        layout.separator()
        layout.label(text="Poses")
        pinlayout = layout.split()
        pc1 = pinlayout.column(align=True)
        pc1.operator('object.store_pose', text='Store Current Pose')
        pc2 = pinlayout.column(align=True)
        pc2.operator('object.load_pose', text='Load Pose')

        #for root in utility.getRoots():
        #    linspect1.operator('object.phobos_select_model', text=root["modelname"]).modelname = \
        #     root["modelname"] if "modelname" in root else root.name


class PhobosModelPanel(bpy.types.Panel):
    """A Custom Panel in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_MODEL_PT_PHOBOS"
    bl_label = "phobos: Object Editing"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon='GROUP')

    def draw(self, context):
        layout = self.layout

        inlayout = layout.split()
        c1 = inlayout.column(align=True)
        c2 = inlayout.column(align=True)
        c1.operator('object.phobos_set_phobostype', text='Set Phobostype')
        c1.operator('object.phobos_sort_objects_to_layers', text="Set Objects to Layers", icon='IMGDISPLAY')
        c1.operator('object.phobos_smoothen_surface', text="Smoothen Surface")
        c1.operator('object.phobos_refine_lod', text="Refine LoD")

        c2.operator('object.phobos_partial_rename', text="Partial Rename")
        c2.operator('object.phobos_batch_property', text='Edit Custom Property', icon='GREASEPENCIL')
        c2.operator('object.phobos_copy_props', text='Copy Custom Property', icon='GHOST')
        c2.operator('object.phobos_rename_custom_property', text='Rename Custom Property', icon='SYNTAX_OFF')
        c2.operator('object.phobos_edityamldictionary', text='Edit Object Dictionary', icon='TEXT')

        layout.separator()
        layout.label(text='Kinematics', icon='POSE_DATA')
        kinlayout = layout.split()
        kc1 = kinlayout.column(align=True)
        kc2 = kinlayout.column(align=True)
        kc1.operator("object.phobos_create_link", text="Create Link(s)")
        kc1.operator('object.create_inertial_objects', text="Create Inertial Object(s)")
        kc1.operator('object.create_collision_objects', text="Create Collision Object(s)")
        kc1.operator('object.phobos_set_origin_to_com', text="Set Origin to COM")
        kc1.operator("object.phobos_create_mimic_joint", text="Mimic Joint")
        kc2.operator('object.define_joint_constraints', text="Define Joint Constraints")
        kc2.operator('object.attach_motor', text="Attach Motor")
        kc2.operator('object.phobos_set_geometry_type', text="Set Geometry Type(s)")
        kc2.operator('object.phobos_set_collision_group', text="Set Collision Group")
        kc2.operator('object.phobos_urdfupdate', text="URDFUpdate")

        #Mass Menu
        layout.separator()
        layout.label(text="Masses & Inertia", icon='PHYSICS')
        minlayout = layout.split()
        mc1 = minlayout.column(align=True)
        mc1.operator('object.phobos_calculate_mass', text='Show Mass')
        mc1.operator('object.phobos_set_mass', text='Set Mass')
        mc2 = minlayout.column(align=True)
        mc2.operator('object.phobos_sync_masses', text='Sync Masses')
        mc2.operator('object.phobos_edit_inertia', text='Edit Inertia')


class PhobosSenConPanel(bpy.types.Panel):
    """A Custom Panel in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_SENCON_PT_PHOBOS"
    bl_label = "phobos: Sensors & Controllers"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon='GAME')

    def draw(self, context):
        slayout = self.layout.split()
        sc1 = slayout.column(align=True)
        # create sensor creation buttons
        #row_sensors.label(text="Add Sensors / Controllers")
        sc1.operator('object.phobos_add_sensor', text="Add/Edit Sensor")
        #sensor_split = row_sensors.split()
        #n_sensortypes = int(len(defs.sensortypes))
        #half_n_sensortypes = int(n_sensortypes / 2)
        #col_sensor_1 = sensor_split.column(align=True)
        #for i in range(half_n_sensortypes):  #sensor in defs.sensorTypes:
        #    sensor = defs.sensortypes[i]
        #    #col_sensor_1.operator('object.phobos_add_sensor_'+sensor, text=sensor)
        #    col_sensor_1.operator('object.phobos_add_sensor', text=sensor).sensor_type = sensor
        #col_sensor_2 = sensor_split.column(align=True)
        #for i in range(n_sensortypes - half_n_sensortypes):
        #    sensor = defs.sensortypes[i + half_n_sensortypes]
        #    col_sensor_2.operator('object.phobos_add_sensor', text=sensor).sensor_type = sensor
        #    #col_sensor_2.operator('object.phobos_add_sensor_'+sensor, text=sensor)
        sc2 = slayout.column(align=True)
        sc2.operator("object.phobos_add_controller", text="Add Controller")


# class PhobosVisPanel(bpy.types.Panel):
#     """A Custom Panel in the Phobos viewport toolbar"""
#     bl_idname = "TOOLS_VIS_PT_PHOBOS"
#     bl_label = "phobos: Visibility"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'TOOLS'
#     bl_category = 'Phobos'
#
#     def draw_header(self, context):
#         self.layout.label(icon = 'VISIBLE_IPO_ON')
#
#     def draw(self, context):
#         lvis = self.layout
#         # Visibility
#         lsplit = lvis.column(align=True)
#         #lsplit = layout.split()
#         lsplit.prop(bpy.data.worlds[0], "showBodies")
#         lsplit.prop(bpy.data.worlds[0], "showJoints")
#         lsplit.prop(bpy.data.worlds[0], "showJointSpheres")
#         lsplit.prop(bpy.data.worlds[0], "showSensors")
#         lsplit.prop(bpy.data.worlds[0], "showDecorations")
#         lsplit.prop(bpy.data.worlds[0], "showConstraints")
#         lsplit.prop(bpy.data.worlds[0], "showNames")
#         lsplit.prop(bpy.data.worlds[0], "showMotorTypes")


class PhobosExportPanel(bpy.types.Panel):
    """A Custom Panel in the Phobos viewport toolbar"""
    bl_idname = "TOOLS_EXPORT_PT_PHOBOS"
    bl_label = "phobos: Export & Import"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon='FILESEL')

    def draw(self, context):
        layout = self.layout

        #export robot model options
        self.layout.label(text="Model Export Settings")
        self.layout.prop(bpy.data.worlds[0], "path")
        ginlayout = self.layout.split()
        g1 = ginlayout.column(align=True)
        g1.prop(bpy.data.worlds[0], "relativePath")
        g1.prop(bpy.data.worlds[0], "structureExport", text="Structure Export")
        g2 = ginlayout.column(align=True)
        g2.prop(bpy.data.worlds[0], "decimalPlaces")

        layout.separator()

        inlayout = self.layout.split()
        c1 = inlayout.column(align=True)
        c1.label(text="Mesh export")
        c1.prop(bpy.data.worlds[0], "exportMeshes", text="Export Meshes")

        c1.prop(bpy.data.worlds[0], "useBobj", text="Use .bobj format")
        c1.prop(bpy.data.worlds[0], "useObj", text="Use .obj format")
        c1.prop(bpy.data.worlds[0], "useStl", text="Use .stl format")
        c1.prop(bpy.data.worlds[0], "useDae", text="Use .dae format")
        if bpy.data.worlds[0].useObj:
            labeltext = ".obj is used"
        elif bpy.data.worlds[0].useBobj:
            labeltext = ".bobj is used"
        elif bpy.data.worlds[0].useStl:
            labeltext = ".stl is used"
        elif bpy.data.worlds[0].useDae:
            labeltext = ".dae is used"
        else:
            labeltext = ".obj is used"
        c1.label(text=labeltext)
        c2 = inlayout.column(align=True)
        c2.label(text="Robot Data Export")
        #c2.prop(bpy.data.worlds[0], "exportMARSscene", text="as MARS scene")
        c2.prop(bpy.data.worlds[0], "exportSMURF", text="As SMURF")
        c2.prop(bpy.data.worlds[0], "exportURDF", text="As URDF")
        c2.prop(bpy.data.worlds[0], "exportSRDF", text="With SRDF")
        c2.prop(bpy.data.worlds[0], "exportYAML", text="As YAML dump")
        c2.prop(bpy.data.worlds[0], "exportTextures", text="Export textures")
        c2.prop(bpy.data.worlds[0], "exportCustomData", text="Export custom data")

        ec1 = layout.column(align=True)
        ec1.operator("object.phobos_export_robot", text="Export Robot Model", icon="PASTEDOWN")
        ec2 = layout.column(align=True)
        ec2.operator("obj.import_robot_model", text="Import Robot Model", icon="COPYDOWN")

        layout.separator()
        layout.label(text="Baking")
        layout.operator("object.phobos_export_bake", text="Bake Robot Model", icon="OUTLINER_OB_ARMATURE")
        layout.operator("object.phobos_create_robot_instance", text="Create Robot Lib Instance", icon="RENDERLAYERS")
        layout.operator('object.phobos_import_lib_robot', text="Import Robot Bake", icon="COPYDOWN")

        layout.separator()

        layout.label(text="Export Scene")
        self.layout.prop(bpy.data.worlds[0], "sceneName", text="Name")
        self.layout.prop(bpy.data.worlds[0], "heightmapMesh", text="export heightmap as mesh")
        layout.operator("object.phobos_export_scene", text="Export SMURF Scene", icon="WORLD_DATA")


class PhobosSettingsPanel(bpy.types.Panel):
    """A Custom Panel in the Viewport Toolbar for Phobos settings"""
    bl_idname = "TOOLS_SETTINGS_PT_PHOBOS"
    bl_label = "phobos: Settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon='MODIFIER')

    def draw(self, context):
        layout = self.layout

        inlayout = layout.split()
        c1 = inlayout.column(align=True)
        c1.operator('object.phobos_adjust_logger', text='Adjust Logging Settings')
        #c2 = inlayout.column(align=True)
        #c2.operator('object.phobos_partial_rename', text="Partial Rename")


class PhobosObjectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_PHOBOS"
    bl_label = "phobos: Object Panel Displaying Custom Properties"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"
    bl_category = 'Phobos'

    def draw_header(self, context):
        self.layout.label(icon='SMOOTH')

    def draw(self, context):

        layout = self.layout

        #the following is used for real pre-defined rather than custom properties
        #row_type.prop(bpy.context.active_object, "type")
        row_type = layout.row()
        row_type.label(icon="OBJECT_DATA")
        #row_type.prop_enum(bpy.context.active_object, '["type"]', "node")
        #row_type.prop_enum(bpy.context.active_object, 'phobostype')
        row_type.prop(bpy.context.active_object, 'phobostype')

        box_props = layout.box()

        try:
            for prop in defs.type_properties[bpy.context.active_object.phobostype]:
                #box_props.label(prop)
                if prop in bpy.context.active_object:
                    box_props.prop(bpy.context.active_object, '["' + prop + '"]')
        except KeyError:
            print("Key could not be found.")
            #else:
            #    bpy.context.active_object[prop] = defs.type_properties[bpy.context.active_object.phobostype+"_default"]


# if script is run directly, register contained classes
if __name__ == "__main__":
    register()
